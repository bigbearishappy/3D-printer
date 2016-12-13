#include"Marlin.h"
#include"planner.h"
#include "stepper.h"
#include"temperature.h"
#include<string.h>
#include "language.h"

float axis_steps_per_unit[4];
float max_feedrate[4];
unsigned long max_acceleration_units_per_sq_second[4];

unsigned long max_acceleration_units_per_sq_second[4]; 	// Use M201 to override by software
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

float acceleration;										// Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
float retract_acceleration;								//  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
float minimumfeedrate;
float minsegmenttime;
float mintravelfeedrate;
float max_xy_jerk;
float max_z_jerk;
float max_e_jerk;

#ifdef AUTOTEMP
float autotemp_max=250;
float autotemp_min=210;
float autotemp_factor=0.1;
int autotemp_enabled=false;
#endif

//private value
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           	// Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           	// Index of the block to process now

// The current position of the tool in absolute steps
long position[4];   									//rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[4]; 						// Speed of previous path line segment
volatile static float previous_nominal_speed; 					// Nominal speed of previous path line segment

unsigned long axis_steps_per_sqr_second[NUM_AXIS];

/****************************************************************************
name:		plan_init
function:	
			initialize the planner with the struct block_t
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void plan_init(void)
{
	block_buffer_head = 0;
  	block_buffer_tail = 0;
  	memset(position, 0, sizeof(position)); // clear position
  	previous_speed[0] = 0.0;
  	previous_speed[1] = 0.0;
  	previous_speed[2] = 0.0;
  	previous_speed[3] = 0.0;
  	previous_nominal_speed = 0.0;
}

// Returns the index of the previous block in the ring buffer
/***************************************************************************************************
name:		prev_block_index()
function:	return the index of the previous block
			[in]	-	block_index:the current block's index
			[out]	-	int8_t:the previous block's index
***************************************************************************************************/
static int8_t prev_block_index(int8_t block_index) 
{
  if (block_index == 0) { 
    block_index = BLOCK_BUFFER_SIZE; 
  }
  block_index--;
  return(block_index);
}

/****************************************************************************
name:		plan_init
function:	
			initialize the planner with the struct block_t
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
uint8_t blocks_queued(void)
{
	if(block_buffer_head == block_buffer_tail){ 
    return false; 
  	}
  	else
    return true;
}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
/***************************************************************************************************
name:		estimate_acceleration_distance()
function:	calculate the distance it takes to accelerate from initial_rate to target_rate by using
			the given acceleration
			[in]	-	initial_rate:initial rate
						target_rate:target rate
						acceleration:the acceleration
			[out]	-	float:the total distance
***************************************************************************************************/
float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration!=0) {
    return((target_rate*target_rate-initial_rate*initial_rate)/
      (2.0*acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}

/***************************************************************************************************
name:		max_allowable_speed()
function:	calculate the maximum allowable speed at this point
			[in]	-	acceleration
						target_velocity
						distance
			[out]	-	float:the speed
***************************************************************************************************/
float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

/***************************************************************************************************
name:		intersection_distance()
function:	calculate the point which you must start braking
			[in]	-	initial_rate
						final_rate
						acceleration
						distance
			[out]	-	the point to start braking
***************************************************************************************************/
float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
  if (acceleration!=0) {
    return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
      (4.0*acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
/***************************************************************************************************
name:		calculate_trapezoid_for_block()
function:	unknow
			[in]	-	
			[out]	-	
***************************************************************************************************/
void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor)
{
  unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)
  long acceleration;		  
  int32_t accelerate_steps;
  int32_t decelerate_steps;
  int32_t plateau_steps;

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if(initial_rate <120) {
    initial_rate=120; 
  }
  if(final_rate < 120) {
    final_rate=120;  
  }

  acceleration = block->acceleration_st;
  accelerate_steps =
    ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

  // Calculate the size of Plateau of Nominal Rate.
  plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps,block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }

  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if(block->busy == false) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps+plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;

}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
/***************************************************************************************************
name:		planner_reverse_pass_kernel()
function:	when the scanning to the last,it helps to go to the first entry
			[in]	-	*previous
						*current
						*next
			[out]	-	void
***************************************************************************************************/
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { 
    return; 
  }

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed,
        max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } 
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
/***************************************************************************************************
name:		planner_reverse_pass()
function:	reverse pass
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
void planner_reverse_pass() 
{
  uint8_t block_index = block_buffer_head;
  unsigned char tail;
  block_t *block[3] = {NULL, NULL, NULL};;
  
  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  tail = block_buffer_tail;
  CRITICAL_SECTION_END;
  
  if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    while(block_index != tail) { 
      block_index = prev_block_index(block_index); 
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
/***************************************************************************************************
name:		planner_forward_pass_kernel()
function:	it helps to go from first to last entry when scanning
			[in]	-	*previous
						*current
						*next
			[out]	-	void
***************************************************************************************************/
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!previous) { 
    return; 
  }

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed,
      max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
/***************************************************************************************************
name:		planner_forward_pass()
function:	forward pass
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = { NULL, NULL, NULL};

  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
/***************************************************************************************************
name:		planner_recalculate_trapezoids()
function:	recalculate the trapezoid speed profiles for all blocks.
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
void planner_recalculate_trapezoids()
{
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
        next->entry_speed/current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }

  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if(next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
    MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}

/***************************************************************************************************
name:		planner_recalculate()
function:	recalculate the motion plan
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
void planner_recalculate() {   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

/****************************************************************************
name:		plan_buffer_line
function:	
			//向数组中添加了一个新的线性移动。step_x,_y,_z是以毫米为单位的绝对位置。毫秒数指定了运动的时间长度。
			//为了达到目标速度，这个计算中也必须提供以毫米为单位的加速过程中运动的距离
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
#ifdef PREVENT_DANGEROUS_EXTRUDE
float extrude_min_temp=EXTRUDE_MINTEMP;
#endif
void plan_buffer_line(const float x, const float y, const float z, const float e, float feed_rate, const uint8_t extruder)
{
	// Calculate the buffer head after we push this byte
	int next_buffer_head ;
	long target[4];
	float inverse_millimeters;
	float inverse_second;
	int moves_queued;
	block_t *block;
	float delta_mm[4];
	volatile unsigned long segment_time = 0;
  	float current_speed[4];
  	float speed_factor = 1.0; //factor <=1 do decrease speed
	int i;
	float max_feedrate[4]; 									// set the max speeds
	float steps_per_mm;
	float vmax_junction;
	float vmax_junction_factor;
	float safe_speed;
	static float previous_nominal_speed; 					// Nominal speed of previous path line segment
	volatile float jerk;
	double v_allowable;

	next_buffer_head = next_block_index(block_buffer_head);

  	// If the buffer is full: good! That means we are well ahead of the robot. 
  	// Rest here until there is room in the buffer.
  	while(block_buffer_tail == next_buffer_head)
  	{
    	//manage_heater(); 
    	//manage_inactivity(); 
    	//lcd_update();
  	}

  	// The target position of the tool in absolute steps
  	// Calculate target position in absolute steps
  	//this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  	target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  	target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  	target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  	target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);

    #ifdef PREVENT_DANGEROUS_EXTRUDE
    if(target[E_AXIS] != position[E_AXIS])
    {
      if(degHotend(active_extruder)<extrude_min_temp)//温度不够，无法进行挤出操作
      {
        position[E_AXIS]=target[E_AXIS]; //behave as if the move really took place, but ignore E part
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
      }
    
      #ifdef PREVENT_LENGTHY_EXTRUDE
      if(labs(target[E_AXIS]-position[E_AXIS])>axis_steps_per_unit[E_AXIS]*EXTRUDE_MAXLENGTH)
      {
        position[E_AXIS]=target[E_AXIS]; //behave as if the move really took place, but ignore E part
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
      }
      #endif
    }
    #endif

	//prepare to set up new block
	block = &block_buffer[block_buffer_head];

	// Mark block as not busy (Not executed by the stepper interrupt)
  	block->busy = false;

  	// Number of steps for each axis
	#ifndef COREXY
	// default non-h-bot planning
	block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
	block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
	#else
	// corexy planning
	// these equations follow the form of the dA and dB equations on http://www.corexy.com/theory.html
	block->steps_x = labs((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]));
	block->steps_y = labs((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]));
	#endif
	block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
  	block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
  	block->steps_e *= extrudemultiply;
  	block->steps_e /= 100;
	block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

	// Bail if this is a zero-length block
  	if (block->step_event_count <= dropsegments)
  	{ 
    	return; 
  	}

  	block->fan_speed = fanSpeed;
  	#ifdef BARICUDA
  	block->valve_pressure = ValvePressure;
  	block->e_to_p_pressure = EtoPPressure;
  	#endif

  	// Compute direction bits for this block 
  	block->direction_bits = 0;
#ifndef COREXY
  if (target[X_AXIS] < position[X_AXIS])
  {
    block->direction_bits |= (1<<X_AXIS); 
  }
  if (target[Y_AXIS] < position[Y_AXIS])
  {
    block->direction_bits |= (1<<Y_AXIS); 
  }
#else
  if ((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]) < 0)
  {
    block->direction_bits |= (1<<X_AXIS); 
  }
  if ((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]) < 0)
  {
    block->direction_bits |= (1<<Y_AXIS); 
  }
#endif

  	if (target[Z_AXIS] < position[Z_AXIS])
  	{
	    block->direction_bits |= (1<<Z_AXIS); 
  	}
  	if (target[E_AXIS] < position[E_AXIS])
  	{
	    block->direction_bits |= (1<<E_AXIS); 
  	}

	block->active_extruder = extruder;

  	//enable active axes
  	#ifdef COREXY
  	if((block->steps_x != 0) || (block->steps_y != 0))
  	{
    	enable_x();
    	enable_y();
  	}
  	#else
  	if(block->steps_x != 0) enable_x();
  	if(block->steps_y != 0) enable_y();
  	#endif
#ifndef Z_LATE_ENABLE
  	if(block->steps_z != 0) enable_z();
#endif

  	// Enable all
  	if(block->steps_e != 0)
  	{
    enable_e0();
    //enable_e1();
    //enable_e2(); 
  	}

  if(block->steps_e == 0)
  {
    if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
  }
  else
  {
    if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
  } 

#ifndef COREXY
    delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
#else
    delta_mm[X_AXIS] = ((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]))/axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = ((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]))/axis_steps_per_unit[Y_AXIS];
#endif

  	delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/axis_steps_per_unit[Z_AXIS];
  	delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS])*extrudemultiply/100.0;
  	if( block->steps_x <=dropsegments && block->steps_y <=dropsegments && block->steps_z <=dropsegments )
  	{
    	block->millimeters = fabs(delta_mm[E_AXIS]);
  	} 
  	else
  	{
	    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  	}

	inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides 

    // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  	inverse_second = feed_rate * inverse_millimeters;

  	moves_queued = (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);

// slow down when de buffer starts to empty, rather than wait at the corner for a buffer refill
#ifdef SLOWDOWN
  	//  segment time im micro seconds
  	segment_time = lround(1000000.0/inverse_second);
  	if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5)))
  	{
    if (segment_time < minsegmenttime)
    { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
      inverse_second=1000000.0/(segment_time+lround(2*(minsegmenttime-segment_time)/moves_queued));
      #ifdef XY_FREQUENCY_LIMIT
         segment_time = lround(1000000.0/inverse_second);
      #endif
    }
  	}
#endif

	block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
	block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  	// Calculate and limit speed in mm/sec for each axis
  	//float current_speed[4];
  	//float speed_factor = 1.0; //factor <=1 do decrease speed
  	for(i=0; i < 4; i++)
  	{
    	current_speed[i] = delta_mm[i] * inverse_second;
    	if(fabs(current_speed[i]) > max_feedrate[i])
      		speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  	}

  	// Correct the speed  
  	if( speed_factor < 1.0)
  	{
	    for(i=0; i < 4; i++)
    	{
      	current_speed[i] *= speed_factor;
    	}
    	block->nominal_speed *= speed_factor;
    	block->nominal_rate *= speed_factor;
  	}

  // Compute and limit the acceleration rate for the trapezoid generator.  
  steps_per_mm = block->step_event_count / block->millimeters;
  if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else
  {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    // Limit acceleration per axis
    if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }
  block->acceleration = block->acceleration_st / steps_per_mm;
  //block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));
  block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (1.0 / 8.0)));

  // Start with a safe speed
  vmax_junction = max_xy_jerk/2; 
  vmax_junction_factor = 1.0; 
  if(fabs(current_speed[Z_AXIS]) > max_z_jerk/2) 
    vmax_junction = min(vmax_junction, max_z_jerk/2);
  if(fabs(current_speed[E_AXIS]) > max_e_jerk/2) 
    vmax_junction = min(vmax_junction, max_e_jerk/2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
    //    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/jerk);
    } 
    if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor= min(vmax_junction_factor, (max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    } 
    if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    } 
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) { 
    block->nominal_length_flag = true; 
  }
  else { 
    block->nominal_length_flag = false; 
  }
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;

  calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed, safe_speed/block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

  planner_recalculate();

  st_wake_up();

}

/***************************************************************************************************
name:		plan_set_a_position()
function:	set the extruder's position
			[in]	-	e:extruder
			[out]	-	void
***************************************************************************************************/
void plan_set_e_position(const float e)
{
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
  st_set_e_position(position[E_AXIS]);
}

/***************************************************************************************************
name:		plan_set_position()
function:	set a position
			[in]	-	x:x axis
						y:y axis
						z:z axis
						e:extruder
			[out]	-	void
***************************************************************************************************/
void plan_set_position(const float x, const float y, const float z, const float e)
{
  position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);//lround: four homes five
  position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  position[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
  //st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);
  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
}

#ifdef PREVENT_DANGEROUS_EXTRUDE
/***************************************************************************************************
name:		set_extruder_min_temp()
function:	set the extruder minimum temperature
			[in]	-	temp:temperature
			[out]	-	void
***************************************************************************************************/
void set_extrude_min_temp(float temp)
{
  extrude_min_temp=temp;
}
#endif

/***************************************************************************************************
name:		check_axes_activity()
function:	check if the a axis is active
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
void check_axes_activity()
{
	unsigned char x_active = 0;
	unsigned char y_active = 0;  
	unsigned char z_active = 0;
	unsigned char e_active = 0;
	volatile unsigned char tail_fan_speed = fanSpeed;
	block_t *block;
	uint8_t block_index;

	if(block_buffer_tail != block_buffer_head)
	{
	block_index = block_buffer_tail;
	tail_fan_speed = block_buffer[block_index].fan_speed;
	while(block_index != block_buffer_head)
	{
	  block = &block_buffer[block_index];
	  if(block->steps_x != 0) x_active++;
	  if(block->steps_y != 0) y_active++;
	  if(block->steps_z != 0) z_active++;
	  if(block->steps_e != 0) e_active++;
	  block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
	}
	}

	if((DISABLE_X) && (x_active == 0)) disable_x();
	if((DISABLE_Y) && (y_active == 0)) disable_y();
	if((DISABLE_Z) && (z_active == 0)) disable_z();
	if((DISABLE_E) && (e_active == 0)) disable_e0(); 

#if defined(FAN_PIN)// && FAN_PIN > -1
	#ifdef FAN_SOFT_PWM
	fanSpeedSoftPwm = tail_fan_speed;
	#else
	//analogWrite(FAN_PIN,tail_fan_speed);
	#endif//!FAN_SOFT_PWM
#endif//FAN_PIN > -1

#ifdef AUTOTEMP
  getHighESpeed();
#endif
}

/***************************************************************************************************
name:		getHighESpeed()
function:	get the speed of the extruder
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
#ifdef AUTOTEMP
void getHighESpeed()
{
	volatile static float oldt=0;
	if(!autotemp_enabled){
    return;
  	}
}
#endif



