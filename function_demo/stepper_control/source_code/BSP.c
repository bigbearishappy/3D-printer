/************************************************************************************************
File name:	BSP.c
Function:	the basic definition of the board
Version:	1.0.0
Date:		2016-3-8
************************************************************************************************/
#include"BSP.h"

/************************************************************************************************ 
Name：RCC_Configuration()
Function:	
		  	initialize the system clock
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);

		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);

		RCC_ADCCLKConfig(RCC_PCLK2_Div2);

		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{}
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08)
		{}
	}
}

/************************************************************************************************ 
Name：GPIO_Configuration 
Function:	
		  	the configuration of GPIO PC4
Parameters：
		   	void
Returns：
			void 
Description:
			ADC input is PC4
************************************************************************************************/
void GPIO_Configuration()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PA1 (ADC Channel1) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/************************************************************************************************ 
Name：USART_Configuration 
Function:	
		  	initialize the usart
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void USART_Configuration()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9;		//TX
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  	= GPIO_Pin_10;		//RX	
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;		                           // A10 USART1_Rx  
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;                           //浮空输入-RX
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate   = 115200;	                                //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	                    //8位数据
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;		                //1个停止位
	USART_InitStructure.USART_Parity     = USART_Parity_No;				            //无奇偶校检
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制禁止
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //发送接收使能

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);	
    USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);	                                                    // Enable the USARTx 
}

/************************************************************************************************ 
Name：fputc 
Function:	
		  	send the data to the usart
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
int fputc(int ch, FILE *f)
{
   USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成 USART2 等
   while (!(USART1->SR & USART_FLAG_TXE));
   return (ch);
}

/************************************************************************************************ 
Name：GetKey 
Function:	
		  	receive data from the usart
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
int GetKey (void)  
{
   while (!(USART1->SR & USART_FLAG_RXNE));
   return ((int)(USART1->DR & 0x1FF));
}

/************************************************************************************************ 
Name：delayms 
Function:	
		  	delay
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void delayms(uint32_t num)
{
		
   int i,j;  
   for(j = 0;j < num; j++)
   {
   	i = 1000;
   	while(i) 
   	{ 
    	 i--; 
   	}
   }  
}
