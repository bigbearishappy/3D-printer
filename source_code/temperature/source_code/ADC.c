/************************************************************************************************
File name:	ADC.c
Function:	the realization of ADC function
Version:	1.0.0
Date:		2016-3-14
************************************************************************************************/
#include"ADC.h"

uint16_t hotend_T = 0,bed_T = 0;
uint8_t switch_data = 0;

/************************************************************************************************ 
Name£ºADC_Configuration 
Function:	
		  	initialize the ADC
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
************************************************************************************************/
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);							//open the clock of ADC1

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;								//initialize the parameter of ADC1
  	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfChannel = 1;
  	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);		//configuration the PA1 to measure the hotend
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5);		//configuration the PA2 to measure the bed
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}
