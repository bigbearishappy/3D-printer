#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"

static uint8_t check_endstops = 1;

static volatile uint8_t endstop_x_hit=false;
static volatile uint8_t endstop_y_hit=false;
static volatile uint8_t endstop_z_hit=false;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};

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
//initialize the dir pin
#if defined(X_DIR_PIN)&&X_DIR_PIN > -1
	//set the pin to output
#endif
#if defined(Y_DIR_PIN)&&Y_DIR_PIN > -1
	//set the pin to output
#endif
#if defined(Z_DIR_PIN)&&Z_DIR_PIN > -1
	//set the pin to output
#endif
#if defined(E0_DIR_PIN)&&E0_DIR_PIN > -1
	//set the pin to output
#endif

//initialize the enable pin
#if defined(X_ENABLE_PIN)&&X_ENABLE_PIN > -1
	//set the pin to output
	if(!X_ENABLE_ON)	//set the X_ENABLE_PIN to high
#endif
#if defined(Y_ENABLE_PIN)&&Y_ENABLE_PIN > -1
	//set the pin to output
	if(!Y_ENABLE_ON)	//set the Y_ENABLE_PIN to high
#endif
#if defined(Z_ENABLE_PIN)&&Z_ENABLE_PIN > -1
	//set the pin to output
	if(!Z_ENABLE_ON)	//set the Z_ENABLE_PIN to high
#endif
#if defined(E0_ENABLE_PIN)&&E0_ENABLE_PIN > -1
	//set the pin to output
	if(!E_ENABLE_ON)	//set the E_ENABLE_PIN to high
#endif

//initialize the endstops
#if defined(X_MIN_PIN)&&X_MIN_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_XMIN
	//set the X_MIN_PIN to high
	#endif
#endif
#if defined(Y_MIN_PIN)&&Y_MIN_PIN > -1
	//set the pin to output
	#ifdef ENDSTOPPULLUP_YMIN
	//set the Y_MIN_PIN to high
	#endif
#endif
#if defined(Z_MIN_PIN)&&Z_MIN_PIN > -1
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
#if defined(X_STEP_PIN) && (X_STEP_PIN > -1)
    //SET_OUTPUT(X_STEP_PIN);
    //WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);	
	disable_x();
#endif
#if defined(Y_STEP_PIN) && (Y_STEP_PIN > -1)
    //SET_OUTPUT(Y_STEP_PIN);
    //WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
    disable_y();
#endif
#if defined(Z_STEP_PIN) && (Z_STEP_PIN > -1)
    //SET_OUTPUT(Z_STEP_PIN);
    //WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
    disable_z();
#endif
#if defined(E0_STEP_PIN) && (E0_STEP_PIN > -1)
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

void digipot_init(void)
{
}

void microstep_init(void)
{}


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
while(blocks_queued()) {
    //manage_heater();
    //manage_inactivity();
	}
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
  ENABLE_STEPPER_DRIVER_INTERRUPT();
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
