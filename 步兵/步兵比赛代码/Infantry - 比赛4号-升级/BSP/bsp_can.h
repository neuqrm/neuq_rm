#ifndef __CAN_H
#define	__CAN_H

#include "stm32f4xx.h"




//#define CAN1                       	
#define CAN1_CLK                    	RCC_APB1Periph_CAN1 //|RCC_APB1Periph_CAN2
#define CAN1_RX_IRQ										CAN1_RX0_IRQn
#define CAN1_RX_IRQHandler						CAN1_RX0_IRQHandler

#define CAN1_GPIO_CLK 								RCC_AHB1Periph_GPIOD
#define CAN1_RX_PIN                 	GPIO_Pin_0
#define CAN1_TX_PIN                 	GPIO_Pin_1
#define CAN1_RX_GPIO_PORT          		GPIOD
#define CAN1_TX_GPIO_PORT          		GPIOD
#define CAN1_RX_GPIO_CLK           		RCC_AHB1Periph_GPIOD
#define CAN1_TX_GPIO_CLK           		RCC_AHB1Periph_GPIOD
#define CAN1_AF_PORT                	GPIO_AF_CAN1
#define CAN1_RX_SOURCE              	GPIO_PinSource0
#define CAN1_TX_SOURCE              	GPIO_PinSource1 

//#define CAN1  
#define CAN2_CLK                    	RCC_APB1Periph_CAN2
#define CAN2_RX_IRQ										CAN2_RX0_IRQn
#define CAN2_RX_IRQHandler						CAN2_RX0_IRQHandler

#define CAN2_GPIO_CLK 								RCC_AHB1Periph_GPIOB
#define CAN2_RX_PIN                 	GPIO_Pin_12
#define CAN2_TX_PIN                 	GPIO_Pin_13
#define CAN2_RX_GPIO_PORT          		GPIOB
#define CAN2_TX_GPIO_PORT          		GPIOB
#define CAN2_RX_GPIO_CLK           		RCC_AHB1Periph_GPIOB
#define CAN2_TX_GPIO_CLK           		RCC_AHB1Periph_GPIOB
#define CAN2_AF_PORT                	GPIO_AF_CAN2
#define CAN2_RX_SOURCE              	GPIO_PinSource12
#define CAN2_TX_SOURCE              	GPIO_PinSource13 


   

static void CAN1_GPIO_Config(void);
static void CAN1_NVIC_Config(void);
static void CAN1_Mode_Config(void);
static void CAN1_Filter_Config(void);
void CAN1_Config(void);

static void CAN2_GPIO_Config(void);
static void CAN2_NVIC_Config(void);
static void CAN2_Mode_Config(void);
static void CAN2_Filter_Config(void);
void CAN2_Config(void);
																	 
void Init_RxMes(CanRxMsg *RxMessage);

#endif







