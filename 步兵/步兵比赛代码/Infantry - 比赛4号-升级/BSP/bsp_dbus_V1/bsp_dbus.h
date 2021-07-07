#ifndef _DJI_REMOTE_H
#define _DJI_REMOTE_H

#include "stdint.h"
#include "stm32f4xx.h"

// #pragma pack(1)
// typedef union
// {
// struct
// {
// struct
// {
// uint8_t ch0_h:8; //!< Byte 0
//
// uint8_t ch0_l:3; //!< Byte 1
// uint8_t ch1_h:5;
//
// uint8_t ch1_l:6; //!< Byte 2
// uint8_t ch2_h:2;
//
// uint8_t ch2_m:8; //!< Byte 3
//
// uint8_t ch2_l:1; //!< Byte 4
// uint8_t ch3_h:7;
//
// uint8_t ch3_l:4; //!< Byte 5
// uint8_t s1:2;
// uint8_t s2:2;
// }rc;
//
// struct
// {
// int16_t x; //!< Byte 6-7
// int16_t y; //!< Byte 8-9
// int16_t z; //!< Byte 10-11
// uint8_t press_l; //!< Byte 12
// uint8_t press_r; //!< Byte 13
// }mouse;
//
// struct
// {
// uint16_t v; //!< Byte 14-15
// }key;
//
// uint16_t resv; //!< Byte 16-17
// };
// uint8_t buf[18]; //!< Union --> Byte<0-17>
// }RC_Ctl_Define_t;
//引脚定义
/*******************************************************/
#define DBUS_USART                             USART1
#define DBUS_USART_CLK                         RCC_APB2Periph_USART1
#define DBUS_USART_BAUDRATE                    100000  //串口波特率

#define DBUS_USART_GPIO_PORT               		 GPIOB
#define DBUS_USART_GPIO_CLK              		   RCC_AHB1Periph_GPIOB
#define DBUS_USART_GPIO_PIN                    GPIO_Pin_7
#define DBUS_USART_GPIO_AF                     GPIO_AF_USART1
#define DBUS_USART_GPIO_SOURCE                 GPIO_PinSource7

#define DBUS_USART_DR_BASE              			 (USART1->DR)		
#define DBUS_BUFFER_SIZE               		      18			//接收的数据量
#define DBUS_USART_DMA_CLK           				    RCC_AHB1Periph_DMA2	
#define DBUS_USART_DMA_CHANNEL									DMA_Channel_4
#define DBUS_USART_DMA_STREAM                   DMA2_Stream5
#define DBUS_USART_DMA_FLAG                     DMA_FLAG_TCIF5
#define DBUS_USART_DMA_IT		                    DMA_IT_TCIF5

#define DBUS_USART_IRQHandler                   USART1_IRQHandler
#define DBUS_USART_IRQ													USART1_IRQn

#define DBUS_DMA_IRQHandler											DMA2_Stream5_IRQHandler
#define DBUS_DMA_IRQ														DMA2_Stream5_IRQn
/************************************************************/


/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)
/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
		uint16_t sw;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint16_t v;
	}key;
}RC_Ctl_t;

typedef struct
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;
	uint16_t sw;
}DJi_RC;

extern DJi_RC rc;
extern RC_Ctl_t RC_Ctl;

void Dji_Remote_Init(void);				//遥控器初始化

#endif
