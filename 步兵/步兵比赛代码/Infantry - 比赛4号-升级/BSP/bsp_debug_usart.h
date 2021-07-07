#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "bsp_debug_usart.h"
#include "stm32f4xx.h"
#include <stdio.h>


//引脚定义 
/*******************************************************/
#define DEBUG_USART                             USART2
#define DEBUG_USART_CLK                         RCC_APB1Periph_USART2
#define DEBUG_USART_PeriphClockCmd              RCC_APB1PeriphClockCmd
#define DEBUG_USART_BAUDRATE                    500000  //串口波特率

#define DEBUG_USART_RX_GPIO_PORT                GPIOD
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define DEBUG_USART_RX_PIN                      GPIO_Pin_6
#define DEBUG_USART_RX_AF                       GPIO_AF_USART2
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource6

#define DEBUG_USART_TX_GPIO_PORT                GPIOD
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define DEBUG_USART_TX_PIN                      GPIO_Pin_5
#define DEBUG_USART_TX_AF                       GPIO_AF_USART2
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource5

#define DEBUG_USART_IRQHandler                  USART2_IRQHandler
#define DEBUG_USART_IRQ                 				USART2_IRQn
/************************************************************/

extern void Debug_USART_Config(void);
extern void DEBUG_SendByte(uint8_t ch);
extern void DEBUG_SendString(char *str);
extern void DEBUG_SendHalfWord(uint16_t ch);
extern void DEBUG_SendHalfString(uint8_t ch,uint16_t *number);
extern void Data_Send(unsigned short int *pst);

#endif /* __USART1_H */
