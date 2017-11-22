#include "usartModule.h"

void usartInit(void)
{
	USART_InitTypeDef* USART_InitStruct;
	USART_ClockInitTypeDef* USART_ClockInitStruct;
	
	/* USART_InitStruct members default value */
  USART_InitStruct->USART_BaudRate = 9600;
  USART_InitStruct->USART_WordLength = USART_WordLength_8b;
  USART_InitStruct->USART_StopBits = USART_StopBits_1;
  USART_InitStruct->USART_Parity = USART_Parity_No ;
  USART_InitStruct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	
	USART_Init(USART1, USART_InitStruct);
	
	/* USART_ClockInitStruct members default value */
  USART_ClockInitStruct->USART_Clock = USART_Clock_Enable;
  USART_ClockInitStruct->USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct->USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStruct->USART_LastBit = USART_LastBit_Disable;
	
	USART_ClockInit(USART1, USART_ClockInitStruct);
	
	USART_Cmd(USART1, ENABLE);
}

