#include"TIM.h"

uint32_t tim_millis = 0;

/****************************************************************************
name:		TIM_Configuration
function:	
			initialize the TIM3
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			the time circle of TIM3 is 0.001 second
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

/******************************************************************************
Name£ºPWM_Configuration 
Function:	
		  	configuration the hotend's pwm
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
******************************************************************************/
void PWM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	  
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//3
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1000; 
	TIM_TimeBaseStructure.TIM_Prescaler =0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	/* Output Compare Active Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 

	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); 
	
	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);  
}

/******************************************************************************
Name£ºPWM_Control 
Function:	
		  	pwm control of motor
Parameters£º
		   	[in]	-	hotend1:pwm value of hotend1
						hotend2:pwm value of hotend2
						bed:	pwm value of bed
Returns£º
			void 
Description:
			GPIO defination:
			PB6			hotend1
			PB7			hotend2
			PB8			bed
			PB9			not used
******************************************************************************/
void PWM_Control(int32_t hotend1, int32_t hotend2, int32_t bed)
{
		TIM_SetCompare1(TIM4,hotend1);
		TIM_SetCompare2(TIM4,hotend2);
		TIM_SetCompare3(TIM4,bed);
		//TIM_SetCompare4(TIM4,0);
}

/****************************************************************************
name:		TIM3_Configuration
function:	
			initialize the TIM3
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			the time circle of TIM3 is 0.001 second
****************************************************************************/
void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  	TIM_DeInit(TIM3);

  	TIM_TimeBaseStructure.TIM_Period = 2 - 1;					//0.001ms = 1us      
  	TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1;//36000 - 1;      
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; 
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
  	TIM_ClearFlag(TIM3, TIM_FLAG_Update);

  	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  

  	TIM_Cmd(TIM3, ENABLE);
}
