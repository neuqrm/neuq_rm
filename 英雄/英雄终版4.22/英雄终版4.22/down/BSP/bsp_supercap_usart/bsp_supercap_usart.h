#ifndef __CAP_USART_H
#define	__CAP_USART_H

#include "stm32f4xx.h"
#include <stdio.h>


//引脚定义 
/*******************************************************/
#define CAP_USART                             USART3
#define CAP_USART_CLK                         RCC_APB1Periph_USART3
#define CAP_USART_PeriphClockCmd              RCC_APB1PeriphClockCmd
#define CAP_USART_BAUDRATE                    115200  //串口波特率

#define CAP_USART_RX_GPIO_PORT                GPIOD
#define CAP_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define CAP_USART_RX_PIN                      GPIO_Pin_9
#define CAP_USART_RX_AF                       GPIO_AF_USART3
#define CAP_USART_RX_SOURCE                   GPIO_PinSource9

#define CAP_USART_TX_GPIO_PORT                GPIOD
#define CAP_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define CAP_USART_TX_PIN                      GPIO_Pin_8
#define CAP_USART_TX_AF                       GPIO_AF_USART3
#define CAP_USART_TX_SOURCE                   GPIO_PinSource8 

#define CAP_USART_IRQHandler                  USART3_IRQHandler
#define CAP_USART_IRQ                 				USART3_IRQn
/************************************************************/

extern void CAP_USART_Config(void);
extern void CAP_SendByte(uint8_t ch);
extern void CAP_SendString(char *str);
extern void CAP_SendHalfWord(uint16_t ch);
extern void CAP_SendHalfString(uint8_t ch,uint16_t *number);
extern void Data_Send(unsigned short int *pst);
extern void use_supercap(void);
extern void supercap_Init(void);
#endif /* __CAP_USART_H */
