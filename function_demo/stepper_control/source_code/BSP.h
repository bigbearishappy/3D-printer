#ifndef BSP_H
#define BSP_H
#include"stm32f10x.h"
#include"stdio.h"

#define WRITE(GPIO_Pin, status)	status = 1?GPIO_SetBits(GPIOA,GPIO_Pin):GPIO_ResetBits(GPIOA,GPIO_Pin)

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);

int fputc(int ch, FILE *f);
int GetKey (void);

void delayms(uint32_t num);

#endif
