#ifndef planner_h
#define planner_h

#include"Marlin.h"

#define max(a,b)	a>b?a:b
#define min(a,b)	a>b?b:a
#define square(a)	a*a

extern float axis_steps_per_unit[4];
extern float max_feedrate[4];
extern unsigned long max_acceleration_units_per_sq_second[4];
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];

extern float acceleration;										// Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
extern float retract_acceleration;								//  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
extern float minimumfeedrate;
extern float minsegmenttime;
extern float mintravelfeedrate;
extern float max_xy_jerk;
extern float max_z_jerk;
extern float max_e_jerk; 

extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail; 

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder
  #ifdef ADVANCE
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  	// Fields used by the motion planner to manage acceleration
	//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec 
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block  
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned long fan_speed;
  #ifdef BARICUDA
  unsigned long valve_pressure;
  unsigned long e_to_p_pressure;
  #endif
  volatile char busy;
} block_t;

#ifdef AUTOTEMP
    extern int autotemp_enabled;
    extern float autotemp_max;
    extern float autotemp_min;
    extern float autotemp_factor;
#endif

void plan_init(void);
static int8_t prev_block_index(int8_t block_index);
uint8_t blocks_queued(void);
float max_allowable_speed(float acceleration, float target_velocity, float distance);
void set_extrude_min_temp(float temp);
float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration);
float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance);
void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor);
void planner_reverse_pass(void);
void planner_forward_pass(void);
void planner_recalculate(void);
void planner_recalculate_trapezoids(void);
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next);
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next);
void plan_buffer_line(const float x, const float y, const float z, const float e, float feed_rate, const uint8_t extruder);
void plan_set_e_position(const float e);
void plan_set_position(const float x, const float y, const float z, const float e);

#endif
