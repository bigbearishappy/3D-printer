#include"USART.h"

uint8_t USART1_Cache[MAX_CMD_SIZE];
uint8_t USART1_DATA_OK = UNREADY;
uint8_t i = 0;
uint8_t comment_mode = 0;

/****************************************************************************
name:		USART_Configuration
function:	
			initialize the usart
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void USART_Configuration(void)
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

void USART1_IRQHandler(void)
{
	uint8_t data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART1_DATA_OK = UNREADY;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		data = USART_ReceiveData(USART1);
		USART1_Cache[i++] = data;
		if(data == '\n' || data == '\r' || (data == ':' && comment_mode == 0) || i >= (MAX_CMD_SIZE - 1) ) 		//receive the end of the cmd
		{
			USART1_DATA_OK = READY;
			i = 0;
		}
	}
}
