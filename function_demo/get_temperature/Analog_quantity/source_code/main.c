/************************************************************************************************
File name:	main.c
Function:	the main function
Version:	1.0.0
Date:		2016-3-8
************************************************************************************************/
/*********************************************HEAD FILE*****************************************/
#include"stm32f10x.h"
#include"BSP.h"
#include"ADC.h"

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
	uint16_t Status;
	uint16_t DataValue;
	RCC_Configuration();
	ADC_Configuration();
	USART_Configuration();
	while(1)
	{
		delayms(5000);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		Status = ADC_GetSoftwareStartConvStatus(ADC1);
		printf("s = %d\r\n", Status);
		DataValue = ADC_GetConversionValue(ADC1);
		printf("adc = %d\r\n", DataValue);
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	}

}
