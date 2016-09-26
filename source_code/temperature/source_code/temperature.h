#ifndef temperature_h
#define temperature_h

#include"Marlin.h"

//FUNCTION
void tp_init(void);

void updatePID(void);
static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
float degHotend(uint8_t extruder);

#ifdef PIDTEMP
  extern float Kp,Ki,Kd,Kc;
  float scalePID_i(float i);
  float scalePID_d(float d);
  float unscalePID_i(float i);
  float unscalePID_d(float d);
#endif

#endif