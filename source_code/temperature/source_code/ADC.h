#ifndef ADC_H
#define ADC_H

#include"stm32f10x.h"

extern uint16_t hotend_T,bed_T;
extern uint8_t switch_data;

void ADC_Configuration(void);

#endif
