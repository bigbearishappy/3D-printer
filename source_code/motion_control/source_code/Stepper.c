#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"

static uint8_t check_endstops = 1;

static volatile uint8_t endstop_x_hit=false;
static volatile uint8_t endstop_y_hit=false;
static volatile uint8_t endstop_z_hit=false;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};

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
while(blocks_queued()) {
    manage_heater();
    manage_inactivity();
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
	//SERIAL_ECHOLN("");
	endstop_x_hit=false;
	endstop_y_hit=false;
	endstop_z_hit=false;
	}
}

