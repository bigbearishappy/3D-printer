#ifndef BSP_H
#define BSP_H
#include"stm32f10x.h"
#include"stdio.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);

int fputc(int ch, FILE *f);
int GetKey (void);

void delayms(uint32_t num);

#endif
