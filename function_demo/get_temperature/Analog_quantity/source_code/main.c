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

float analog2temp(int raw, uint8_t e);

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
	float temperature = 0.0;
	RCC_Configuration();
	ADC_Configuration();
	USART_Configuration();
	while(1)
	{
		delayms(5000);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		Status = ADC_GetSoftwareStartConvStatus(ADC1);
		//printf("s = %d\r\n", Status);
		DataValue = ADC_GetConversionValue(ADC1);
		printf("adc = %d\r\n", DataValue);		

		temperature = analog2temp(DataValue,0);
		printf("temp=%f\r\n", temperature);		
		
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	}

}

#define EXTRUDERS 1
#define OVERSAMPLENR 16
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0
float analog2temp(int raw, uint8_t e)
{
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}