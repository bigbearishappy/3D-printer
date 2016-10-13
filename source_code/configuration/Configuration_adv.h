#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

#define LOW 	0
#define HIGH	1

//Feedrate for manual moves along X, Y, Z, E
#define DEFAULT_MINIMUMFEEDRATE			0.0
#define DEFAULT_MINTRAVELFEEDRATE		0.0

#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME			20000

#define EXTRUDERS						1//the printer is with one extruder

#define MAX_CMD_SIZE					96
#define BUFSIZE							4

#define AUTOTEMP
#ifdef AUTOTEMP
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif

#define PIDTEMP

#ifdef PIDTEMP
// this adds an experimental additional term to the heatingpower, proportional to the extrusion speed.
// if Kc is choosen well, the additional required power due to increased melting should be compensated.
  #define PID_ADD_EXTRUSION_RATE
  #ifdef PID_ADD_EXTRUSION_RATE
    #define  DEFAULT_Kc (1) //heatingpower=Kc*(e_speed)
  #endif  
#endif

#define AXIS_RELATIVE_MODES {false, false, false, false}

#if defined SDSUPPORT
  #define BLOCK_BUFFER_SIZE 16   // SD,LCD,Buttons take more memory, block buffer needs to be smaller
#else
  #define BLOCK_BUFFER_SIZE 16 // maximize block buffer
#endif

//const unsigned int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement
#define dropsegments	5

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

//default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 60

#define ULTIPANEL
//adds support for experimental filament exchange support M600; requires display
#ifdef ULTIPANEL
  #define FILAMENTCHANGEENABLE
  #ifdef FILAMENTCHANGEENABLE
    #define FILAMENTCHANGE_XPOS 3
    #define FILAMENTCHANGE_YPOS 3
    #define FILAMENTCHANGE_ZADD 10
    #define FILAMENTCHANGE_FIRSTRETRACT -2
    #define FILAMENTCHANGE_FINALRETRACT -100
  #endif
#endif

#define BED_HYSTERESIS 2 //only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

// MS1 MS2 Stepper Driver Microstepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#define MICROSTEP16 HIGH,HIGH

#endif

