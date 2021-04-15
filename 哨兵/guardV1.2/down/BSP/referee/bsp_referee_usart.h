#ifndef _REFEREE_USART_H
#define _REFEREE_USART_H

#include "stdint.h"
#include "stm32f4xx.h"

/******************使用链表存储裁判系统数据**************
双向链表 用于缓冲裁判系统的不定时数据
********************************************************/
#define REFEREE_DATA_LENTH	10 //暂存N个数据包
#define HEADSOF		0xA5					//数据帧起始字节
#define REFEREE_BUFFER_LENGTH 						128		//DMA缓存区长度
#define REFEREE_BUFFER_ALLLENGTH					1024  //暂存数据组总长度
//引脚定义 USART6使用DMA的通道5 数据流1
/*******************************************************/
#define REFEREE_USART                             USART3
#define REFEREE_USART_CLK                   			RCC_APB1Periph_USART3
#define REFEREE_USART_BAUDRATE                    115200  //串口波特率

#define REFEREE_USART_GPIO_PORT               		GPIOD
#define REFEREE_USART_GPIO_CLK              		  RCC_AHB1Periph_GPIOD
#define REFEREE_USART_GPIO_PIN                    GPIO_Pin_9
#define REFEREE_USART_GPIO_AF                     GPIO_AF_USART3
#define REFEREE_USART_GPIO_SOURCE                 GPIO_PinSource9

#define REFEREE_USART_TX_GPIO_PORT                GPIOD
#define REFEREE_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define REFEREE_USART_TX_PIN                      GPIO_Pin_8
#define REFEREE_USART_TX_AF                       GPIO_AF_USART3
#define REFEREE_USART_TX_SOURCE                   GPIO_PinSource8


#define REFEREE_USART_DR_BASE              			 	(USART3->DR)		
#define REFEREE_BUFFER_SIZE               		    128		//接收的数据量
#define REFEREE_USART_DMA_CLK           				  RCC_AHB1Periph_DMA1	
#define REFEREE_USART_DMA_CHANNEL									DMA_Channel_4
#define REFEREE_USART_DMA_STREAM                  DMA1_Stream1
#define REFEREE_USART_DMA_FLAG                    DMA_FLAG_TCIF1
#define REFEREE_USART_DMA_IT		                  DMA_IT_TCIF1

#define REFEREE_USART_IRQHandler                  USART3_IRQHandler
#define REFEREE_USART_IRQ													USART3_IRQn

#define REFEREE_DMA_IRQHandler										DMA1_Stream1_IRQHandler
#define REFEREE_DMA_IRQ														DMA1_Stream1_IRQn
/************************************************************/

extern volatile uint16_t Rereree_Datalen;
extern volatile uint16_t Rereree_Data_Bufferlen;
extern uint16_t 	Referee_Data_Length[REFEREE_DATA_LENTH];
extern uint8_t 		Referee_Data_Buffer[REFEREE_BUFFER_ALLLENGTH]; //相当于一个数据栈 用来暂存来不及处理的数据

void Referee_USART_Config(void);				//裁判系统串口初始化
void RefereeDataProcess(vu8 *pData,uint16_t Data_Length);
uint8_t  Referee_DataBuffer_INC(uint8_t *point,uint16_t length);	//数据堆栈添加数据操作
void Referee_DataBuffer_DEC(uint8_t *pData,uint16_t buflen,uint16_t datalen); //数据堆栈取数据操作
void referee_usart(void);

#endif /**_REFEREE_USART_H**/
