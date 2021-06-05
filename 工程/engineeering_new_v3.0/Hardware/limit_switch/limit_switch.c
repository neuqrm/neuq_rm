#include "limit_switch.h"
#include "stm32f4xx.h"

void limit_switch_NVIC_Configuration()//PC2,PC3
{
	 NVIC_InitTypeDef NVIC_InitStructure;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	 NVIC_Init(&NVIC_InitStructure);
 }

 void limit_switch_init()//限位开关外部中断
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 EXTI_InitTypeDef EXTI_InitStructure;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(GPIOC, &GPIO_InitStructure);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource2);
	 EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	 GPIO_Init(GPIOC, &GPIO_InitStructure);
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource3);
	 EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
 }
