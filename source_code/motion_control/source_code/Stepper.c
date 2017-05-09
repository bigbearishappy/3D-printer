#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"

#define WRITE(pin, val)	GPIO_SetBits(GPIOB, GPIO_Pin_2)

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
bool X_MIN_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
bool Y_MIN_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
bool Z_MIN_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
bool X_MAX_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
bool Y_MAX_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
bool Z_MAX_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.

//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
/*static */long counter_x,       // Counter variables for the bresenham line tracer
            counter_y,
            counter_z,
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block

volatile static uint8_t check_endstops = 1;

static volatile uint8_t endstop_x_hit=false;
static volatile uint8_t endstop_y_hit=false;
static volatile uint8_t endstop_z_hit=false;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};

static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[3]={0,0,0};

static bool old_x_min_endstop=false;
static bool old_x_max_endstop=false;
static bool old_y_min_endstop=false;
static bool old_y_max_endstop=false;
static bool old_z_min_endstop=false;
static bool old_z_max_endstop=false;

volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

#define CHECK_ENDSTOPS  if(check_endstops)

#ifdef MOTOR_CURRENT_PWM_XY_PIN
  int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
#endif

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  1

/****************************************************************************
name:		st_init
function:	
			initialize the stepper driver
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void st_init(void)
{
    digipot_init(); //Initialize Digipot Motor Current
    microstep_init(); //Initialize Microstepping Pins

//initialize the dir pin
#if defined(X_DIR_PIN)//&&X_DIR_PIN > -1
	//set the pin to output
#endif
#if defined(Y_DIR_PIN)//&&Y_DIR_PIN > -1
	//set the pin to output
#endif
#if defined(Z_DIR_PIN)//&&Z_DIR_PIN > -1
	//set the pin to output
#endif
#if defined(E0_DIR_PIN)//&&E0_DIR_PIN > -1
	//set the pin to output
#endif

//initialize the enable pin
#if defined(X_ENABLE_PIN)//&&X_ENABLE_PIN > -1
	//set the pin to output
	if(!X_ENABLE_ON)	//set the X_ENABLE_PIN to high
#endif
#if defined(Y_ENABLE_PIN)//&&Y_ENABLE_PIN > -1
	//set the pin to output
	if(!Y_ENABLE_ON)	//set the Y_ENABLE_PIN to high
#endif
#if defined(Z_ENABLE_PIN)//&&Z_ENABLE_PIN > -1
	//set the pin to output
	if(!Z_ENABLE_ON)	//set the Z_ENABLE_PIN to high
#endif
#if defined(E0_ENABLE_PIN)//&&E0_ENABLE_PIN > -1
	//set the pin to output
	if(!E_ENABLE_ON)	//set the E_ENABLE_PIN to high
#endif

//initialize the endstops
#if defined(X_MIN_PIN)//&&X_MIN_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_XMIN
	//set the X_MIN_PIN to high
	#endif
#endif
#if defined(Y_MIN_PIN)//&&Y_MIN_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_YMIN
	//set the Y_MIN_PIN to high
	#endif
#endif
#if defined(Z_MIN_PIN)//&&Z_MIN_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_ZMIN
	//set the Z_MIN_PIN to high
	#endif
#endif

#if defined(X_MAX_PIN)&&X_MAX_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_XMAX
	//set the X_MAX_PIN to high
	#endif
#endif
#if defined(Y_MAX_PIN)&&Y_MAX_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_YMAX
	//set the Y_MAX_PIN to high
	#endif
#endif
#if defined(Z_MAX_PIN)&&Z_MAX_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_ZMAX
	//set the Z_MAX_PIN to high
	#endif
#endif

//initialize the step pin
#if defined(X_STEP_PIN)// && (X_STEP_PIN > -1)
    //SET_OUTPUT(X_STEP_PIN);
    //WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);	
	disable_x();
#endif
#if defined(Y_STEP_PIN)// && (Y_STEP_PIN > -1)
    //SET_OUTPUT(Y_STEP_PIN);
    //WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
    disable_y();
#endif
#if defined(Z_STEP_PIN)// && (Z_STEP_PIN > -1)
    //SET_OUTPUT(Z_STEP_PIN);
    //WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
    disable_z();
#endif
#if defined(E0_STEP_PIN)// && (E0_STEP_PIN > -1)
    //SET_OUTPUT(E0_STEP_PIN);
    //WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e0();
#endif

//ENABLE_STEPPER_DRIVER_INTERRUPT();

enable_endstops(1); // Start with endstops active. After homing they can be disabled
}

/****************************************************************************
name:		endstops_hit_on_purpose
function:	
			make the endstops unable to be hit
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void endstops_hit_on_purpose(void)
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

/****************************************************************************
name:		enable_endstops
function:	
			enable the endstops
Parameters:
			[in]	-	uint8_t check:the value of the check_endstops
Returns:
			[out]	-	void
Description:
			Start with endstops active. After homing they can be disabled
****************************************************************************/
void enable_endstops(uint8_t check)
{
	check_endstops = check;
}

/*****************************************************************************************************
name:		digipot_init()
function:	initialize the digital pot
			[in]	-	void
			[out]	-	void
*****************************************************************************************************/
void digipot_init(void)
{
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
    int i;
	const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;
	
	//SPI.begin();
	//pinMode(DIGIPOTSS_PIN, OUTPUT);
	for(i=0;i<=4;i++)
	  digipot_current(i,digipot_motor_current[i]);
#endif

#ifdef MOTOR_CURRENT_PWM_XY_PIN
	//pinMode(MOTOR_CURRENT_PWM_XY_PIN, OUTPUT);
	//pinMode(MOTOR_CURRENT_PWM_Z_PIN, OUTPUT);
	//pinMode(MOTOR_CURRENT_PWM_E_PIN, OUTPUT);
	digipot_current(0, motor_current_setting[0]);
	digipot_current(1, motor_current_setting[1]);
	digipot_current(2, motor_current_setting[2]);
	//Set timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
	//TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
#endif
}

/**********************************************************************************************************
name:		microstep_init()
function:	initialize the microstep
			[in]	-	void
			[out]	-	void
**********************************************************************************************************/
void microstep_init(void)
{
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
    int i;
	const uint8_t microstep_modes[] = MICROSTEP_MODES;
	//pinMode(X_MS2_PIN,OUTPUT);
	//pinMode(Y_MS2_PIN,OUTPUT);
	//pinMode(Z_MS2_PIN,OUTPUT);
	//pinMode(E0_MS2_PIN,OUTPUT);
	//pinMode(E1_MS2_PIN,OUTPUT);
	for(i=0;i<=4;i++) 
		microstep_mode(i,microstep_modes[i]);
#endif
}

/**************************************************************************************************
name:		microstep_ms()
function:	manage all the microstep of the printer
			[in]	-	driver
						ms1
						ms2
			[out]	-	void
**************************************************************************************************/
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2)
{
  if(ms1 > -1) switch(driver)
  {
    case 0: 
	//digitalWrite( X_MS1_PIN,ms1); 
	break;
    case 1: 
	//digitalWrite( Y_MS1_PIN,ms1); 
	break;
    case 2: 
	//digitalWrite( Z_MS1_PIN,ms1);
	break;
    case 3: 
	//digitalWrite(E0_MS1_PIN,ms1); 
	break;
    case 4: 
	//digitalWrite(E1_MS1_PIN,ms1); 
	break;
  }
  if(ms2 > -1) switch(driver)
  {
    case 0: 
	//digitalWrite( X_MS2_PIN,ms2); 
	break;
    case 1: 
	//digitalWrite( Y_MS2_PIN,ms2); 
	break;
    case 2: 
	//digitalWrite( Z_MS2_PIN,ms2); 
	break;
    case 3: 
	//digitalWrite(E0_MS2_PIN,ms2); 
	break;
    case 4: 
	//digitalWrite(E1_MS2_PIN,ms2); 
	break;
  }
}

/*********************************************************************************************************
name:		microstep_mode()
function:	set the mode of a microstep
			[in]	-	driver
						stepping_mode
			[out]	-	void
*********************************************************************************************************/
void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{
	switch(stepping_mode)
	{
	case 1: microstep_ms(driver,MICROSTEP1); break;
	case 2: microstep_ms(driver,MICROSTEP2); break;
	case 4: microstep_ms(driver,MICROSTEP4); break;
	case 8: microstep_ms(driver,MICROSTEP8); break;
	case 16: microstep_ms(driver,MICROSTEP16); break;
	}
}

/****************************************************************************
name:		st_synchronize
function:	
			check the block queue and executed it
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void st_synchronize(void)
{
/*while(blocks_queued()) {
    manage_heater();
    manage_inactivity();
	}
	*/
}

/***************************************************************************************************
name:		st_wake_up()
function:	wake up the stepper
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
void st_wake_up(void)
{
  //  TCNT1 = 0;
  //ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**************************************************************************************************
name:		st_set_e_position()
function:	set the position of the extruder
			[in]	-	&e:extruder value
**************************************************************************************************/
void st_set_e_position(const long e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

/***************************************************************************************************
name:		finishAndDisableSteppers()
function:	check the command and disable all the steppers
			[in]	-	void
			[out]	-	void
***************************************************************************************************/
void finishAndDisableSteppers()
{
  st_synchronize();
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  //disable_e1();
  //disable_e2();
}

/*******************************************************************************************************
name:		digipot_current()
function:	write the value to the pot
			[in]	-	driver
						current
			[out]	-	void
*******************************************************************************************************/
void digipot_current(uint8_t driver, int current)
{}

/****************************************************************************************************
name:		checkHitEndstops()
function:	check if the endstops is hit
			[in]	-	void
			[out]	-	void
****************************************************************************************************/
void checkHitEndstops()
{
	if(endstop_x_hit || endstop_y_hit || endstop_z_hit) {
	//SERIAL_ECHO_START;
	//SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
	if(endstop_x_hit) {
	 //SERIAL_ECHOPAIR(" X:",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
	}
	if(endstop_y_hit) {
	 //SERIAL_ECHOPAIR(" Y:",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
	}
	if(endstop_z_hit) {
	 //SERIAL_ECHOPAIR(" Z:",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
	}
	SERIAL_ECHOLN("");
	endstop_x_hit=false;
	endstop_y_hit=false;
	endstop_z_hit=false;
	}
}

unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer = 50;
//   if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

//   if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
//     step_rate = (step_rate >> 2)&0x3fff;
//     step_loops = 4;
//   }
//   else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
//     step_rate = (step_rate >> 1)&0x7fff;
//     step_loops = 2;
//   }
//   else {
//     step_loops = 1;
//   }

//   if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
//   step_rate -= (F_CPU/500000); // Correct for minimal speed
//   if(step_rate >= (8*256)){ // higher step rate
//     unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
//     unsigned char tmp_step_rate = (step_rate & 0x00ff);
//     unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
//     MultiU16X8toH16(timer, tmp_step_rate, gain);
//     timer = (unsigned short)pgm_read_word_near(table_address) - timer;
//   }
//   else { // lower step rates
//     unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
//     table_address += ((step_rate)>>1) & 0xfffc;
//     timer = (unsigned short)pgm_read_word_near(table_address);
//     timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
//   }
//   if(timer < 100) { timer = 100; MYSERIAL.print(MSG_STEPPER_TOO_HIGH); MYSERIAL.println(step_rate); }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
/****************************************************************************************************
name:		trapezoid_generator_reset()
function:	initialize the trapezoid generator
			[in]	-	void
			[out]	-	void
****************************************************************************************************/
void trapezoid_generator_reset() {
  #ifdef ADVANCE
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
    old_advance = advance >>8;
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  //OCR1A = acceleration_time;  //set timer interrupt time

//    SERIAL_ECHO_START;
//    SERIAL_ECHOPGM("advance :");
//    SERIAL_ECHO(current_block->advance/256.0);
//    SERIAL_ECHOPGM("advance rate :");
//    SERIAL_ECHO(current_block->advance_rate/256.0);
//    SERIAL_ECHOPGM("initial advance :");
//  SERIAL_ECHO(current_block->initial_advance/256.0);
//    SERIAL_ECHOPGM("final advance :");
//    SERIAL_ECHOLN(current_block->final_advance/256.0);

}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
void TIM3_IRQHandler(void)
{
	unsigned short timer;
  unsigned short step_rate;
	int8_t i = 0;
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		//ISR code
		
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;

      #ifdef Z_LATE_ENABLE
        if(current_block->steps_z > 0) {
          enable_z();
          OCR1A = 2000; //1ms wait
          return;
        }
      #endif

//      #ifdef ADVANCE
//      e_steps[current_block->active_extruder] = 0;
//      #endif
    }
    else {
        //OCR1A=2000; // 1kHz.
				TIM_SetAutoreload(TIM3, 2000);
				TIM_SetCounter(TIM3, 0);
				TIM_Cmd(TIM3, ENABLE);
    }
  }
	
	if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;


    // Set the direction bits (X_AXIS=A_AXIS and Y_AXIS=B_AXIS for COREXY)
    if((out_bits & (1<<X_AXIS))!=0){
      #ifdef DUAL_X_CARRIAGE
        if (extruder_duplication_enabled){
          WRITE(X_DIR_PIN, INVERT_X_DIR);
          WRITE(X2_DIR_PIN, INVERT_X_DIR);
        }
        else{
          if (current_block->active_extruder != 0)
            //WRITE(X2_DIR_PIN, INVERT_X_DIR);
						GPIO_SetBits(GPIOB, GPIO_Pin_2);
          else
						GPIO_SetBits(GPIOB, GPIO_Pin_2);
            //WRITE(X_DIR_PIN, INVERT_X_DIR);
        }
      #else
        ////WRITE(X_DIR_PIN, INVERT_X_DIR);
				GPIO_SetBits(GPIOB, GPIO_Pin_2);
      #endif        
      count_direction[X_AXIS]=-1;
    }
    else{
      #ifdef DUAL_X_CARRIAGE
        if (extruder_duplication_enabled){
          WRITE(X_DIR_PIN, !INVERT_X_DIR);
          WRITE(X2_DIR_PIN, !INVERT_X_DIR);
        }
        else{
          if (current_block->active_extruder != 0)
            WRITE(X2_DIR_PIN, !INVERT_X_DIR);
          else
            WRITE(X_DIR_PIN, !INVERT_X_DIR);
        }
      #else
        ////WRITE(X_DIR_PIN, !INVERT_X_DIR);
				GPIO_SetBits(GPIOB, GPIO_Pin_2);
      #endif        
      count_direction[X_AXIS]=1;
    }
	
    if((out_bits & (1<<Y_AXIS))!=0){
      ////WRITE(Y_DIR_PIN, INVERT_Y_DIR);
	  
	  #ifdef Y_DUAL_STEPPER_DRIVERS
	    WRITE(Y2_DIR_PIN, !(INVERT_Y_DIR == INVERT_Y2_VS_Y_DIR));
	  #endif
	  
      count_direction[Y_AXIS]=-1;
    }
    else{
      ////WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
	  
	  #ifdef Y_DUAL_STEPPER_DRIVERS
	    WRITE(Y2_DIR_PIN, (INVERT_Y_DIR == INVERT_Y2_VS_Y_DIR));
	  #endif
	  
      count_direction[Y_AXIS]=1;
    }

    // Set direction en check limit switches
    #ifndef COREXY
    if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
    #else
    if ((((out_bits & (1<<X_AXIS)) != 0)&&(out_bits & (1<<Y_AXIS)) != 0)) {   //-X occurs for -A and -B
    #endif
      CHECK_ENDSTOPS
      {
        #ifdef DUAL_X_CARRIAGE
        // with 2 x-carriages, endstops are only checked in the homing direction for the active extruder
        if ((current_block->active_extruder == 0 && X_HOME_DIR == -1) 
            || (current_block->active_extruder != 0 && X2_HOME_DIR == -1))
        #endif          
        {
          #if defined(X_MIN_PIN) && X_MIN_PIN > -1
            bool x_min_endstop=(READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING);
            if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
              endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
              endstop_x_hit=true;
              step_events_completed = current_block->step_event_count;
            }
            old_x_min_endstop = x_min_endstop;
          #endif
        }
      }
    }
    else { // +direction
      CHECK_ENDSTOPS
      {
        #ifdef DUAL_X_CARRIAGE
        // with 2 x-carriages, endstops are only checked in the homing direction for the active extruder
        if ((current_block->active_extruder == 0 && X_HOME_DIR == 1) 
            || (current_block->active_extruder != 0 && X2_HOME_DIR == 1))
        #endif          
        {
          #if defined(X_MAX_PIN) && X_MAX_PIN > -1
            bool x_max_endstop=(READ(X_MAX_PIN) != X_MAX_ENDSTOP_INVERTING);
            if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
              endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
              endstop_x_hit=true;
              step_events_completed = current_block->step_event_count;
            }
            old_x_max_endstop = x_max_endstop;
          #endif
        }
      }
    }

    #ifndef COREXY
    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
    #else
    if ((((out_bits & (1<<X_AXIS)) != 0)&&(out_bits & (1<<Y_AXIS)) == 0)) {   // -Y occurs for -A and +B
    #endif
      CHECK_ENDSTOPS
      {
        #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
          bool y_min_endstop=(READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING);
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_min_endstop = y_min_endstop;
        #endif
      }
    }
    else { // +direction
      CHECK_ENDSTOPS
      {
        #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
          bool y_max_endstop=(READ(Y_MAX_PIN) != Y_MAX_ENDSTOP_INVERTING);
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_max_endstop = y_max_endstop;
        #endif
      }
    }

    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      ////WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      
      #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,INVERT_Z_DIR);
      #endif

      count_direction[Z_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
          bool z_min_endstop=(READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
    }
    else { // +direction
      ////WRITE(Z_DIR_PIN,!INVERT_Z_DIR);

      #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,!INVERT_Z_DIR);
      #endif

      count_direction[Z_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
          bool z_max_endstop=(READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING);
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_max_endstop = z_max_endstop;
        #endif
      }
    }

    #ifndef ADVANCE
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
        REV_E_DIR();
        count_direction[E_AXIS]=-1;
      }
      else { // +direction
        NORM_E_DIR();
        count_direction[E_AXIS]=1;
      }
    #endif //!ADVANCE



    for(i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves)
      #ifndef AT90USB
      //MSerial.checkRx(); // Check for serial chars.
      #endif

      #ifdef ADVANCE
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        counter_e -= current_block->step_event_count;
        if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
          e_steps[current_block->active_extruder]--;
        }
        else {
          e_steps[current_block->active_extruder]++;
        }
      }
      #endif //ADVANCE

        counter_x += current_block->steps_x;
        if (counter_x > 0) {
        #ifdef DUAL_X_CARRIAGE
          if (extruder_duplication_enabled){
            WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
            WRITE(X2_STEP_PIN, !INVERT_X_STEP_PIN);
          }
          else {
            if (current_block->active_extruder != 0)
              //WRITE(X2_STEP_PIN, !INVERT_X_STEP_PIN);
						GPIO_SetBits(GPIOB, GPIO_Pin_2);
            else
              WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          }
        #else
          //WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
					GPIO_SetBits(GPIOB, GPIO_Pin_2);
        #endif        
          counter_x -= current_block->step_event_count;
          count_position[X_AXIS]+=count_direction[X_AXIS];   
        #ifdef DUAL_X_CARRIAGE
          if (extruder_duplication_enabled){
            WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
            WRITE(X2_STEP_PIN, INVERT_X_STEP_PIN);
          }
          else {
            if (current_block->active_extruder != 0)
              WRITE(X2_STEP_PIN, INVERT_X_STEP_PIN);
            else
              WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
          }
        #else
          //WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
					GPIO_SetBits(GPIOB, GPIO_Pin_2);
        #endif
        }

        counter_y += current_block->steps_y;
        if (counter_y > 0) {
					GPIO_SetBits(GPIOB, GPIO_Pin_2);
          ////WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
		  
		  #ifdef Y_DUAL_STEPPER_DRIVERS
			////WRITE(Y2_STEP_PIN, !INVERT_Y_STEP_PIN);
					GPIO_SetBits(GPIOB, GPIO_Pin_2);
		  #endif
		  
          counter_y -= current_block->step_event_count;
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          ////WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
					GPIO_SetBits(GPIOB, GPIO_Pin_2);
		  
		  #ifdef Y_DUAL_STEPPER_DRIVERS
			WRITE(Y2_STEP_PIN, INVERT_Y_STEP_PIN);
		  #endif
        }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        //WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
				GPIO_SetBits(GPIOB, GPIO_Pin_2);
        
        #ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, !INVERT_Z_STEP_PIN);
        #endif

        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        ////WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
				GPIO_SetBits(GPIOB, GPIO_Pin_2);
        
        #ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, INVERT_Z_STEP_PIN);
        #endif
      }

      #ifndef ADVANCE
        counter_e += current_block->steps_e;
        if (counter_e > 0) {
          WRITE_E_STEP(!INVERT_E_STEP_PIN);
          counter_e -= current_block->step_event_count;
          count_position[E_AXIS]+=count_direction[E_AXIS];
          WRITE_E_STEP(INVERT_E_STEP_PIN);
        }
      #endif //!ADVANCE
      step_events_completed += 1;
      if(step_events_completed >= current_block->step_event_count) break;
    }
    // Calculare new timer value
//     unsigned short timer;
//     unsigned short step_rate;
    if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {

      ////MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;

      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      //OCR1A = timer;
			
				TIM_SetAutoreload(TIM3, 2000);
				TIM_SetCounter(TIM3, 0);
				TIM_Cmd(TIM3, ENABLE);
			
      acceleration_time += timer;
      #ifdef ADVANCE
        for(i=0; i < step_loops; i++) {
          advance += advance_rate;
        }
        //if(advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;

      #endif
    }
    else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {
      ////MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if(step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      //OCR1A = timer;
			
			TIM_SetAutoreload(TIM3, 2000);
			TIM_SetCounter(TIM3, 0);
			TIM_Cmd(TIM3, ENABLE);
			
      deceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance -= advance_rate;
        }
        if(advance < final_advance) advance = final_advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;
      #endif //ADVANCE
    }
    else {
      //OCR1A = OCR1A_nominal;
			
				TIM_SetAutoreload(TIM3, 2000);
				TIM_SetCounter(TIM3, 0);
				TIM_Cmd(TIM3, ENABLE);
			
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
			}
		}
	}
	}
// 		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
// 	} 
// }
