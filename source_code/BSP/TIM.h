#ifndef TIM_H
#define TIM_H
#include"Marlin.h"

extern uint32_t tim_millis;

void TIM_Configuration(void);

void PWM_Configuration(void);
void PWM_Control(int32_t hotend1, int32_t hotend2, int32_t bed);

#endif
