#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include"Configuration_adv.h"

#define BAUDRATE 115200

#define PIDTEMP
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current

//#define ULTIPANEL

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef MOTHERBOARD
#define MOTHERBOARD 33
#endif

//
#define  DEFAULT_Kp 22.2
#define  DEFAULT_Ki 1.08
#define  DEFAULT_Kd 114

#define PID_INTEGRAL_DRIVE_MAX			255								//limit for the integral term
#define PID_dT 							10								//((16.0 * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine

//default setting
#define DEFAULT_AXIS_STEPS_PER_UNIT 	{78.7402,78.7402,2560,760*1.1}  //default steps per unit
#define DEFAULT_MAX_FEEDRATE			{500, 500, 5, 25}    			// (mm/sec)
#define DEFAULT_MAX_ACCELERATION		{9000,9000,100,10000}			// X, Y, Z, E maximum start speed for accelerated moves.

#define DEFAULT_ACCELERATION			3000							// X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION	3000							// X, Y, Z and E max acceleration in mm/s^2 for retracts

//MOVEMENT SETTINGS
#define NUM_AXIS 4						//The axis order in all axis related arrays is X, Y, Z, E;

//the speed change that does not require acceleration.(the speed change instantaneously)
#define DEFAULT_XYJERK					20.0							//(mm/sec)
#define DEFAULT_ZJERK					0.4								//(mm/sec)
#define DEFAULT_EJERK					5.0								//(mm/sec)

//Preheat settings
#define PLA_PREHEAT_HOTEND_TEMP			180
#define PLA_PREHEAT_HPB_TEMP			70
#define PLA_PREHEAT_FAN_SPEED			255								//BASE ON THE PWM

#define ABS_PREHEAT_HOTEND_TEMP			240
#define ABS_PREHEAT_HPB_TEMP			100
#define ABS_PREHEAT_FAN_SPEED			255								//BASE ON THE PWM

#define HEATER_0_MINTEMP				5

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing
#define X_MAX_POS 205
#define X_MIN_POS 0
#define Y_MAX_POS 205
#define Y_MIN_POS 0
#define Z_MAX_POS 200
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

// these are the offsets to the prob relative to the extruder tip (Hotend - Probe)
#define X_PROBE_OFFSET_FROM_EXTRUDER -25
#define Y_PROBE_OFFSET_FROM_EXTRUDER -29
#define Z_PROBE_OFFSET_FROM_EXTRUDER -12.35

#define Z_RAISE_BEFORE_HOMING 4       // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                      // Be sure you have this distance over your Z_MAX_POS in case

#define XY_TRAVEL_SPEED 8000         // X and Y axis travel speed between probes, in mm/min

//If you have enabled the Bed Auto Levelling and are using the same Z Probe for Z Homing,
//it is highly recommended you let this Z_SAFE_HOMING enabled!!!

#define Z_SAFE_HOMING   // This feature is meant to avoid Z homing with probe outside the bed area.
                          // When defined, it will:
                          // - Allow Z homing only after X and Y homing AND stepper drivers still enabled
                          // - If stepper drivers timeout, it will need X and Y homing again before Z homing
                          // - Position the probe in a defined XY point before Z Homing when homing all axis (G28)
                          // - Block Z homing only when the probe is outside bed area.

#ifdef Z_SAFE_HOMING
	#define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)    // X point for Z homing when homing all axis (G28)
	#define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)    // Y point for Z homing when homing all axis (G28)
#endif

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE

#define EXTRUDE_MINTEMP 170

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

//#define TEMP_SENSOR_0 -1
#define TEMP_SENSOR_0 3
//#define TEMP_SENSOR_1 -1
#define TEMP_SENSOR_1 1
//#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_2 0
//#define TEMP_SENSOR_BED 0
#define TEMP_SENSOR_BED 3

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275
#define BED_MAXTEMP 150

#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders

#endif
