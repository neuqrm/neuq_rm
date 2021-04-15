#ifndef _REFEREE_USART_H
#define _REFEREE_USART_H

#include "stdint.h"
#include "stm32f4xx.h"

/******************ʹ������洢����ϵͳ����**************
˫������ ���ڻ������ϵͳ�Ĳ���ʱ����
********************************************************/
#define REFEREE_DATA_LENTH	10 //�ݴ�N�����ݰ�
#define HEADSOF		0xA5					//����֡��ʼ�ֽ�
#define REFEREE_BUFFER_LENGTH 						128		//DMA����������
#define REFEREE_BUFFER_ALLLENGTH					1024  //�ݴ��������ܳ���
//���Ŷ��� USART6ʹ��DMA��ͨ��5 ������1
/*******************************************************/
#define REFEREE_USART                             USART3
#define REFEREE_USART_CLK                   			RCC_APB1Periph_USART3
#define REFEREE_USART_BAUDRATE                    115200  //���ڲ�����

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
#define REFEREE_BUFFER_SIZE               		    128		//���յ�������
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
extern uint8_t 		Referee_Data_Buffer[REFEREE_BUFFER_ALLLENGTH]; //�൱��һ������ջ �����ݴ����������������

void Referee_USART_Config(void);				//����ϵͳ���ڳ�ʼ��
void RefereeDataProcess(vu8 *pData,uint16_t Data_Length);
uint8_t  Referee_DataBuffer_INC(uint8_t *point,uint16_t length);	//���ݶ�ջ������ݲ���
void Referee_DataBuffer_DEC(uint8_t *pData,uint16_t buflen,uint16_t datalen); //���ݶ�ջȡ���ݲ���
void referee_usart(void);

#endif /**_REFEREE_USART_H**/
