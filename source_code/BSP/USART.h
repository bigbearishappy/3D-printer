#ifndef USART_H
#define USART_H
#include"stm32f10x.h"
#include"Configuration_adv.h"
#include"Marlin.h"
#include"queue_loop.h"

#define UNREADY	0	
#define READY	1	

extern char USART1_Cache[MAX_CMD_SIZE];
extern Queue_l *queue;

extern uint8_t USART1_DATA_OK;
extern uint8_t comment_mode;

void USART_Configuration(void);

#endif
