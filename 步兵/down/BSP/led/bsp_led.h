#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

//引脚定义
/*******************************************************/
#define LED_GPIO_CLK             RCC_AHB1Periph_GPIOG
//自定义绿色灯1
#define LED1_PIN                  GPIO_Pin_1                 
#define LED1_GPIO_PORT            GPIOG                      
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯2
#define LED2_PIN                  GPIO_Pin_2                 
#define LED2_GPIO_PORT            GPIOG                      
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯3
#define LED3_PIN                  GPIO_Pin_3                
#define LED3_GPIO_PORT            GPIOG                       
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯4
#define LED4_PIN                  GPIO_Pin_4                 
#define LED4_GPIO_PORT            GPIOG                       
#define LED4_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯5
#define LED5_PIN                  GPIO_Pin_5                 
#define LED5_GPIO_PORT            GPIOG                      
#define LED5_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯6
#define LED6_PIN                  GPIO_Pin_6                 
#define LED6_GPIO_PORT            GPIOG                      
#define LED6_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯7
#define LED7_PIN                  GPIO_Pin_7                 
#define LED7_GPIO_PORT            GPIOG                       
#define LED7_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯8
#define LED8_PIN                  GPIO_Pin_8                 
#define LED8_GPIO_PORT            GPIOG                       
#define LED8_GPIO_CLK             RCC_AHB1Periph_GPIOG

//自定义绿色灯 G
#define LEDG_PIN                  GPIO_Pin_14                 
#define LEDG_GPIO_PORT            GPIOF                       
#define LEDG_GPIO_CLK             RCC_AHB1Periph_GPIOF

//自定义红色灯 R
#define LEDR_PIN                  GPIO_Pin_11                 
#define LEDR_GPIO_PORT            GPIOE                       
#define LEDR_GPIO_CLK             RCC_AHB1Periph_GPIOE

//自定义激光
#define LASER_PIN                  GPIO_Pin_13                 
#define LASER_GPIO_PORT            GPIOG                       
#define LASER_GPIO_CLK             RCC_AHB1Periph_GPIOG
/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define LED_ON  0
#define LED_ 1

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (~a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2(a)	if (~a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_PIN)

#define LED3(a)	if (~a)	\
					GPIO_SetBits(LED3_GPIO_PORT,LED3_PIN);\
					else		\
					GPIO_ResetBits(LED3_GPIO_PORT,LED3_PIN)
					
#define LED4(a)	if (~a)	\
					GPIO_SetBits(LED4_GPIO_PORT,LED4_PIN);\
					else		\
					GPIO_ResetBits(LED4_GPIO_PORT,LED4_PIN)
					
#define LED5(a)	if (~a)	\
					GPIO_SetBits(LED5_GPIO_PORT,LED5_PIN);\
					else		\
					GPIO_ResetBits(LED5_GPIO_PORT,LED5_PIN)

#define LED6(a)	if (~a)	\
					GPIO_SetBits(LED6_GPIO_PORT,LED6_PIN);\
					else		\
					GPIO_ResetBits(LED6_GPIO_PORT,LED6_PIN)

#define LED7(a)	if (~a)	\
					GPIO_SetBits(LED7_GPIO_PORT,LED7_PIN);\
					else		\
					GPIO_ResetBits(LED7_GPIO_PORT,LED7_PIN)
					
#define LED8(a)	if (~a)	\
					GPIO_SetBits(LED8_GPIO_PORT,LED8_PIN);\
					else		\
					GPIO_ResetBits(LED8_GPIO_PORT,LED8_PIN)
					
#define LEDG(a)	if (~a)	\
					GPIO_SetBits(LEDG_GPIO_PORT,LEDG_PIN);\
					else		\
					GPIO_ResetBits(LEDG_GPIO_PORT,LEDG_PIN)
					
#define LEDR(a)	if (~a)	\
					GPIO_SetBits(LEDR_GPIO_PORT,LEDR_PIN);\
					else		\
					GPIO_ResetBits(LEDR_GPIO_PORT,LEDR_PIN)

#define LASER(a)	if (~a)	\
					GPIO_SetBits(LASER_GPIO_PORT,LASER_PIN);\
					else		\
					GPIO_ResetBits(LASER_GPIO_PORT,LASER_PIN)
/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRRL=i;}			  //设置为高电平		
#define digitalLo(p,i)			{p->BSRRH=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态


/* 定义控制IO的宏 */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON		digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF		digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE		digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON		digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF		digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE		digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON		digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF		digitalLo(LED3_GPIO_PORT,LED3_PIN)

#define LED4_TOGGLE		digitalToggle(LED4_GPIO_PORT,LED4_PIN)
#define LED4_ON		digitalHi(LED4_GPIO_PORT,LED4_PIN)
#define LED4_OFF		digitalLo(LED4_GPIO_PORT,LED4_PIN)

#define LED5_TOGGLE		digitalToggle(LED5_GPIO_PORT,LED5_PIN)
#define LED5_ON		digitalHi(LED5_GPIO_PORT,LED5_PIN)
#define LED5_OFF		digitalLo(LED5_GPIO_PORT,LED5_PIN)

#define LED6_TOGGLE		digitalToggle(LED6_GPIO_PORT,LED6_PIN)
#define LED6_ON		digitalHi(LED6_GPIO_PORT,LED6_PIN)
#define LED6_OFF		digitalLo(LED6_GPIO_PORT,LED6_PIN)

#define LED7_TOGGLE		digitalToggle(LED7_GPIO_PORT,LED7_PIN)
#define LED7_ON		digitalHi(LED7_GPIO_PORT,LED7_PIN)
#define LED7_OFF			digitalLo(LED7_GPIO_PORT,LED7_PIN)

#define LED8_TOGGLE		digitalToggle(LED8_GPIO_PORT,LED8_PIN)
#define LED8_ON		digitalHi(LED8_GPIO_PORT,LED8_PIN)
#define LED8_OFF			digitalLo(LED8_GPIO_PORT,LED8_PIN)

#define LEDG_TOGGLE		digitalToggle(LEDG_GPIO_PORT,LEDG_PIN)
#define LEDG_ON		digitalHi(LEDG_GPIO_PORT,LEDG_PIN)
#define LEDG_OFF		digitalLo(LEDG_GPIO_PORT,LEDG_PIN)

#define LEDR_TOGGLE		digitalToggle(LEDR_GPIO_PORT,LEDR_PIN)
#define LEDR_ON		digitalHi(LEDR_GPIO_PORT,LEDR_PIN)
#define LEDR_OFF			digitalLo(LEDR_GPIO_PORT,LEDR_PIN)

#define LASER_TOGGLE		digitalToggle(LASER_GPIO_PORT,LASER_PIN)
#define LASER_ON		digitalHi(LASER_GPIO_PORT,LASER_PIN)
#define LASER_OFF			digitalLo(LASER_GPIO_PORT,LASER_PIN)

void LED_Black(void);
void LED_Bright(void);
void LED_Toggle(void);
void LED_Control(uint8_t select);
void LED_GPIO_Config(void);

#endif /* __LED_H */
