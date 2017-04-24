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
	USART_Configuration();
	while(1)
	{
		delayms(50);
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
		delayms(50);
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	}

}

void GPIOA_Configuration()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitTypeStruct);

	GPIO_SetBits(GPIOA, GPIO_Pin_0);
}
