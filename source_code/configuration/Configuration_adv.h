#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

//Feedrate for manual moves along X, Y, Z, E
#define DEFAULT_MINIMUMFEEDRATE			0.0
#define DEFAULT_MINTRAVELFEEDRATE		0.0

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME			20000

#define EXTRUDERS						1//the printer is with one extruder

#define MAX_CMD_SIZE					96
#define BUFSIZE							4

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

#endif

