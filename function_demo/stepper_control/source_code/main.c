/************************************************************************************************
File name:	main.c
Function:	the main function
Version:	1.0.0
Date:		2016-3-8
************************************************************************************************/
/*********************************************HEAD FILE*****************************************/
#include"stm32f10x.h"
#include"BSP.h"

void GPIOA_Configuration(void);
void GPIOB_Configuration(void);
void TIM3_Configuration(void);
int getsteptime(int stepcnt);

int i = 0;
int steptime = 0;
int stepcnt = 0;

/************************************************************************************************ 
Name£ºmain 
Function:	
		  	main
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
************************************************************************************************/
int main()
{
	RCC_Configuration();
	GPIOA_Configuration();
	GPIOB_Configuration();
	USART_Configuration();
	//TIM3_Configuration();
	while(1)
	{
		//delayms(5);
		GPIO_SetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7);
		delayms(10);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7);
//		steptime = getsteptime(stepcnt);
//		stepcnt++;
//		if(stepcnt > 570)
//			stepcnt = 0;
//		delayms(steptime);
		delayms(500);

	}

}

void GPIOA_Configuration()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitTypeStruct);

	//GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

void GPIOB_Configuration()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitTypeStruct);

	GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_5 | GPIO_Pin_6);
}

void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TypeDef *TIM;
	unsigned char IRQ;
	unsigned char pri;

	TIM = TIM3;
	IRQ = TIM3_IRQn;
	pri = 10;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//Timer configuration
	TIM_TimeBaseStructure.TIM_Period = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = 36-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = pri;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(TIM, TIM_FLAG_Update);
	TIM_ITConfig(TIM, TIM_IT_Update, ENABLE);
	//TIM_Cmd(TIM, DISABLE);
	TIM_Cmd(TIM, ENABLE);
}

int getsteptime(int stepcnt)
{
	if(stepcnt < tablen)
		return steptab1[stepcnt]*6;
	else
		return steptab1[tablen - 1]*6;
}

void  TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_Cmd(TIM3, DISABLE);
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		steptime = getsteptime(stepcnt);
		stepcnt++;
		
		GPIO_SetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7);
		for(i = 0; i < 10; i++);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7);
		
		TIM_SetAutoreload(TIM3, steptime/10);
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
	}
}
