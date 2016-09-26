#include"TIM.h"

uint32_t tim_millis = 0;

/****************************************************************************
name:		TIM_Configuration
function:	
			initialize the TIM2
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			the time circle of TIM2 is 0.001 second
****************************************************************************/
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  	TIM_DeInit(TIM2);

  	TIM_TimeBaseStructure.TIM_Period = 2 - 1;					//0.001s      
  	TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;      
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
  	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

  	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  

  	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		tim_millis++;
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
	} 
}
