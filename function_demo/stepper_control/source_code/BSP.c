/************************************************************************************************
File name:	BSP.c
Function:	the basic definition of the board
Version:	1.0.0
Date:		2016-3-8
************************************************************************************************/
#include"BSP.h"

//short steptab1[] = {
//	2614,2500,2426,2272,2144,2035,1941,1859,1786,1721,1663,1610,1561,1517,1477,1439,1404,
//	1372,1342,1313,1287,1261,1238,1215,1194,1174,1155,1136,1119,1102,1086,1071,1056,1042,1029,1016,
//	1003,991,979,968,957,947,936,927,917,908,899,890,882,873,865,857,850,842,835,828,821,815,808,
//	802,796,789,784,778,772,766,761,756,750,745,740,735,731,726,721,717,712,708,704,699,695,691,687,
//	683,679,675,672,668,664,661,657,654,651,647,644,641,637,634,631,628,625,622,619,616,614,611,
//	608,605,603,600,597,595,592,590,587,585,582,580,577,575,573,570,568,558,548,538,529,520,512,
//	504,497,489,482,476,469,463,457,452,446,441,426,422,417,413,409,405,401,397,395,394,393,392,
//	391,390,386,383,379,376,373,370,367,364,361,358,355,353,350,346,341,336,331,326,322,317,313,
//	
//};
//short steptab2[] = {
//	2000/2,	1800/2,	1200/2,	1070/2,	970/2,	920/2,	880/2,
//	830/2,	790/2,	750/2,	710/2,	670/2,	630/2,	580/2, 
//	540/2,	500/2, 	460/2,
//};

short steptab1[] = {
	2000,	1800,	1200,	1070,	970,	920,	880,
	830,	790,	750,	710,	670,	630,	580, 	540,	500, 	460,
};

unsigned int tablen = sizeof(steptab1)/sizeof(steptab1[0]);

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
   	i = 8;
   	while(i) 
   	{ 
    	 i--; 
   	}
   }  
}
