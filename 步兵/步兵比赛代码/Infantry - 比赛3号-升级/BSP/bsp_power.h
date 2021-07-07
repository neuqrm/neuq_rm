#ifndef __CTRL_H
#define	__CTRL_H

#include "stm32f4xx.h"

//引脚定义
/*******************************************************/
#define POWER_GPIO_CLK            RCC_AHB1Periph_GPIOH
#define POWER_GPIO_PORT           GPIOH
#define POWER1_PIN								GPIO_Pin_2   
#define POWER2_PIN								GPIO_Pin_3   
#define POWER3_PIN								GPIO_Pin_4  
#define POWER4_PIN								GPIO_Pin_5   

/***控制电源开关的的宏，
	* 电源低电平亮，设置ON=0，OFF=1
	* 若电源高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define POWER_ON  1
#define POWER_ONOFF 0

/* 带参宏，可以像内联函数一样使用 */
#define POWER1(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER1_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER1_PIN)

#define POWER2(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER2_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER2_PIN)
					
#define POWER3(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER3_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER3_PIN)
					
#define POWER4(a)	if (a)	\
					GPIO_SetBits(POWER_GPIO_PORT,POWER4_PIN);\
					else		\
					GPIO_ResetBits(POWER_GPIO_PORT,POWER4_PIN)
					
					
/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRRL=i;}			  //设置为高电平		
#define digitalLo(p,i)			{p->BSRRH=i;}				//输出低电平


/* 定义控制IO的宏 */
#define POWER1_OFF		digitalHi(POWER_GPIO_PORT,POWER1_PIN)
#define POWER1_ON			digitalLo(POWER_GPIO_PORT,POWER1_PIN)

#define POWER2_OFF		digitalHi(POWER_GPIO_PORT,POWER2_PIN)
#define POWER2_ON			digitalLo(POWER_GPIO_PORT,POWER2_PIN)

#define POWER3_OFF		digitalHi(POWER_GPIO_PORT,POWER3_PIN)
#define POWER3_ON			digitalLo(POWER_GPIO_PORT,POWER3_PIN)

#define POWER4_OFF		digitalHi(POWER_GPIO_PORT,POWER4_PIN)
#define POWER4_ON			digitalLo(POWER_GPIO_PORT,POWER4_PIN)

void POWER_GPIO_Config(void);

#endif /* __CTRL_H */
