#ifndef _REFEREE_USART_H
#define _REFEREE_USART_H

#include "stdint.h"
#include "stm32f4xx.h"

/******************????????›¥??????????**************
??????? ??????????????????????
********************************************************/
#define REFEREE_DATA_LENTH	10 //???N???????
#define HEADSOF		0xA5					//???????????
#define REFEREE_BUFFER_LENGTH 						128		//DMA??????????
#define REFEREE_BUFFER_ALLLENGTH					1024  //??????????????
//??????? USART6???DMA?????5 ??????1
/*******************************************************/
#define REFEREE_USART                             USART6
#define REFEREE_USART_CLK                   			RCC_APB2Periph_USART6
#define REFEREE_USART_BAUDRATE                    115200  //?????????

#define REFEREE_USART_GPIO_PORT               		GPIOG
#define REFEREE_USART_GPIO_CLK              		  RCC_AHB1Periph_GPIOG
#define REFEREE_USART_GPIO_PIN                    GPIO_Pin_9
#define REFEREE_USART_GPIO_AF                     GPIO_AF_USART6
#define REFEREE_USART_GPIO_SOURCE                 GPIO_PinSource9

#define REFEREE_USART_TX_GPIO_PORT                GPIOG
#define REFEREE_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOG
#define REFEREE_USART_TX_PIN                      GPIO_Pin_14
#define REFEREE_USART_TX_AF                       GPIO_AF_UART6
#define REFEREE_USART_TX_SOURCE                   GPIO_PinSource14


#define REFEREE_USART_DR_BASE              			 	(USART6->DR)		
#define REFEREE_BUFFER_SIZE               		    128		//???????????
#define REFEREE_USART_DMA_CLK           				  RCC_AHB1Periph_DMA2	
#define REFEREE_USART_DMA_CHANNEL									DMA_Channel_5
#define REFEREE_USART_DMA_STREAM                  DMA2_Stream1
#define REFEREE_USART_DMA_FLAG                    DMA_FLAG_TCIF1
#define REFEREE_USART_DMA_IT		                  DMA_IT_TCIF1

#define REFEREE_USART_IRQHandler                  USART6_IRQHandler
#define REFEREE_USART_IRQ													USART6_IRQn

#define REFEREE_DMA_IRQHandler										DMA2_Stream1_IRQHandler
#define REFEREE_DMA_IRQ														DMA2_Stream1_IRQn
/************************************************************/

extern volatile uint16_t Rereree_Datalen;
extern volatile uint16_t Rereree_Data_Bufferlen;
extern uint16_t 	Referee_Data_Length[REFEREE_DATA_LENTH];
extern uint8_t 		Referee_Data_Buffer[REFEREE_BUFFER_ALLLENGTH]; //???????????? ???????????????????????

void Referee_USART_Config(void);				//??????????????
void RefereeDataProcess(vu8 *pData,uint16_t Data_Length);
uint8_t  Referee_DataBuffer_INC(uint8_t *point,uint16_t length);	//?????????????????
void Referee_DataBuffer_DEC(uint8_t *pData,uint16_t buflen,uint16_t datalen); //??????????????
#endif /**_REFEREE_USART_H**/
