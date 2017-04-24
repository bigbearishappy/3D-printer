/************************************************************************************************
File name:	ADC.c
Function:	the configuration of the ADC
Version:	1.0.0
Date:		2016-3-8
************************************************************************************************/
#include"stm32f10x.h"
#include"ADC.h"

/************************************************************************************************ 
Name£ºADC_Configuration 
Function:	
		  	the configuration of the ADC
Parameters£º
		   	void
Returns£º
			void 
Description:
			PA1 is used as the input
************************************************************************************************/
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfChannel = 1;
  	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_55Cycles5);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
