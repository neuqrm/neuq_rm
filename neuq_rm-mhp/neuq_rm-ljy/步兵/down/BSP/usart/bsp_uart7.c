/**
  ******************************************************************************
  * @file    bsp_JSON_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_uart7.h"

void JSON_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_AHB1PeriphClockCmd(JSON_USART_RX_GPIO_CLK|JSON_USART_TX_GPIO_CLK,ENABLE);
	JSON_USART_PeriphClockCmd(JSON_USART_CLK, ENABLE);
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = JSON_USART_TX_PIN  ;  
	GPIO_Init(JSON_USART_TX_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = JSON_USART_RX_PIN;
	GPIO_Init(JSON_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(JSON_USART_RX_GPIO_PORT,JSON_USART_RX_SOURCE,JSON_USART_RX_AF);
	GPIO_PinAFConfig(JSON_USART_TX_GPIO_PORT,JSON_USART_TX_SOURCE,JSON_USART_TX_AF);
	USART_InitStructure.USART_BaudRate = JSON_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(JSON_USART, &USART_InitStructure); 
	USART_Cmd(JSON_USART, ENABLE);
	USART_ITConfig(JSON_USART, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = JSON_USART_IRQ ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
}



int fputc(int ch, FILE *f)
{
	USART_SendData(JSON_USART, (uint8_t) ch);
	while (USART_GetFlagStatus(JSON_USART, USART_FLAG_TXE) == RESET);		
	return (ch);
}



int fgetc(FILE *f)
{
	while (USART_GetFlagStatus(JSON_USART, USART_FLAG_RXNE) == RESET);
	return (int)USART_ReceiveData(JSON_USART);
}

/*********************************************END OF FILE**********************/
