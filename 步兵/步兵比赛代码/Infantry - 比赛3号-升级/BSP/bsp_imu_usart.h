#ifndef __IMU_USART_H
#define	__IMU_USART_H

#include "stm32f4xx.h"
#include <stdio.h>


//引脚定义 
/*******************************************************/
#define IMU_USART                             UART8
#define IMU_USART_CLK                         RCC_APB1Periph_UART8
#define IMU_USART_PeriphClockCmd              RCC_APB1PeriphClockCmd
#define IMU_USART_BAUDRATE                    921600  //串口波特率

#define IMU_USART_RX_GPIO_PORT                GPIOE
#define IMU_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOE
#define IMU_USART_RX_PIN                      GPIO_Pin_0
#define IMU_USART_RX_AF                       GPIO_AF_UART8
#define IMU_USART_RX_SOURCE                   GPIO_PinSource0

#define IMU_USART_TX_GPIO_PORT                GPIOE
#define IMU_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOE
#define IMU_USART_TX_PIN                      GPIO_Pin_1
#define IMU_USART_TX_AF                       GPIO_AF_UART8
#define IMU_USART_TX_SOURCE                   GPIO_PinSource1

#define IMU_USART_IRQHandler                  UART8_IRQHandler
#define IMU_USART_IRQ                 				UART8_IRQn
/************************************************************/


extern void IMU_USART_Config(void);
extern void IMU_SendByte(uint8_t ch);
extern void IMU_SendString(char *str);


#endif /* __USART8_H */
