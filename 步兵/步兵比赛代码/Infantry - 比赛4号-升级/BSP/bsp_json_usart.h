#ifndef __JSON_USART_H
#define	__JSON_USART_H

#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"

//���Ŷ��� 
/*******************************************************/
#define JSON_USART                             UART7
#define JSON_USART_CLK                         RCC_APB1Periph_UART7
#define JSON_USART_PeriphClockCmd              RCC_APB1PeriphClockCmd
#define JSON_USART_BAUDRATE                    115200  //���ڲ�����

#define JSON_USART_RX_GPIO_PORT                GPIOE
#define JSON_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOE
#define JSON_USART_RX_PIN                      GPIO_Pin_7
#define JSON_USART_RX_AF                       GPIO_AF_UART7
#define JSON_USART_RX_SOURCE                   GPIO_PinSource7

#define JSON_USART_TX_GPIO_PORT                GPIOE
#define JSON_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOE
#define JSON_USART_TX_PIN                      GPIO_Pin_8
#define JSON_USART_TX_AF                       GPIO_AF_UART7
#define JSON_USART_TX_SOURCE                   GPIO_PinSource8

#define JSON_USART_IRQHandler                  UART7_IRQHandler
#define JSON_USART_IRQ												 UART7_IRQn

void JSON_USART_Config(void);

#endif /* __JSON_USART_H*/


