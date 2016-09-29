#ifndef temperature_h
#define temperature_h

#include"Marlin.h"

extern int target_temperature[EXTRUDERS];

extern int target_temperature_bed;
extern float current_temperature_bed;

//FUNCTION
void tp_init(void);

void updatePID(void);
static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
float degHotend(uint8_t extruder);
float degTargetHotend(uint8_t extruder);
int isHeatingHotend(uint8_t extruder);
int isHeatingBed(void);
int isCoolingBed(void);
void disable_heater(void);
void PID_autotune(float temp, int extruder, int ncycles);

#ifdef PIDTEMP
  extern float Kp,Ki,Kd,Kc;
  float scalePID_i(float i);
  float scalePID_d(float d);
  float unscalePID_i(float i);
  float unscalePID_d(float d);
#endif

#endif
