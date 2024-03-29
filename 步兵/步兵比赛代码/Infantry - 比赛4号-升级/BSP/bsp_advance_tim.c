/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   高级控制定时器互补输出范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_advance_tim.h"

__IO uint16_t ChannePulse = 1000;
__IO uint16_t ChannePulseHero = 1000;
__IO uint16_t ChannePulseInfantry = 400;
uint16_t Helm_pluse = 1000;

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void TIM1_GPIO_Config(void) 
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启定时器相关的GPIO外设时钟*/
	RCC_AHB1PeriphClockCmd (TIM1_PWM12_GPIO_CLOCK , ENABLE); 
	RCC_AHB1PeriphClockCmd (TIM1_PWM34_GPIO_CLOCK , ENABLE); 

	
	/* 定时器功能引脚初始化 */															   
	GPIO_InitStructure.GPIO_Pin = (TIM1_PWM1_PIN | TIM1_PWM2_PIN);	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(TIM1_PWM12_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = (TIM1_PWM3_PIN | TIM1_PWM4_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(TIM1_PWM34_GPIO_PORT, &GPIO_InitStructure);
	
	
	GPIO_PinAFConfig(TIM1_PWM12_GPIO_PORT,GPIO_PinSource8,GPIO_AF_TIM1);
	GPIO_PinAFConfig(TIM1_PWM12_GPIO_PORT,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(TIM1_PWM34_GPIO_PORT,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(TIM1_PWM34_GPIO_PORT,GPIO_PinSource14,GPIO_AF_TIM1);
	

}


static void TIM8_GPIO_Config(void) 
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启定时器相关的GPIO外设时钟*/
	RCC_AHB1PeriphClockCmd (TIM8_PWM_GPIO_CLOCK , ENABLE); 

	
	/* 定时器功能引脚初始化 */															   
	GPIO_InitStructure.GPIO_Pin = (TIM8_PWM1_PIN | TIM8_PWM2_PIN | TIM8_PWM3_PIN | TIM8_PWM4_PIN);	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(TIM8_PWM_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(TIM8_PWM_GPIO_PORT,TIM8_PWM1_SOURCE,GPIO_AF_TIM8);
	GPIO_PinAFConfig(TIM8_PWM_GPIO_PORT,TIM8_PWM2_SOURCE,GPIO_AF_TIM8);
	GPIO_PinAFConfig(TIM8_PWM_GPIO_PORT,TIM8_PWM3_SOURCE,GPIO_AF_TIM8);
	GPIO_PinAFConfig(TIM8_PWM_GPIO_PORT,TIM8_PWM4_SOURCE,GPIO_AF_TIM8);
	
}
/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM1_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// 开启TIMx_CLK,x[1,8] 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

  /* 累计 TIM_Period个后产生一个更新或者中断*/		
  //当定时器从0计数到19999，即为2000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Period = 20000-1;
	// 高级控制定时器时钟源TIMxCLK = HCLK=180MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 180-1;	
  // 采样时钟分频 1分频 即180MHz
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  // 计数方式
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  // 重复计数器
  TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器TIMx, x[1,8]
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
  /*PWM模式配置*/
	//配置为PWM模式1
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  
  TIM_OCInitStructure.TIM_Pulse = ChannePulseHero;	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
  TIM_OCInitStructure.TIM_Pulse = ChannePulseHero;    // 
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
  TIM_OCInitStructure.TIM_Pulse = ChannePulseInfantry;    // 
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
  TIM_OCInitStructure.TIM_Pulse = ChannePulseInfantry;    // 
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	// 使能定时器
	TIM_Cmd(TIM1, ENABLE);	
	
	/* 主动输出使能 */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


static void TIM8_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// 开启TIMx_CLK,x[1,8] 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); 

  /* 累计 TIM_Period个后产生一个更新或者中断*/		
  //当定时器从0计数到19999，即为2000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Period = 20000-1;
	// 高级控制定时器时钟源TIMxCLK = HCLK=180MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 180-1;	
  // 采样时钟分频 2分频 即90MHz
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV4;
  // 计数方式
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  // 重复计数器
  TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器TIMx, x[1,8]
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
  /*PWM模式配置*/
	//配置为PWM模式1
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  
  TIM_OCInitStructure.TIM_Pulse = ChannePulse;	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
  TIM_OCInitStructure.TIM_Pulse = ChannePulse;    // 
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
  TIM_OCInitStructure.TIM_Pulse = ChannePulse;    // 
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
  TIM_OCInitStructure.TIM_Pulse = ChannePulse;    // 
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	// 使能定时器
	TIM_Cmd(TIM8, ENABLE);	
	
	/* 主动输出使能 */
  TIM_CtrlPWMOutputs(TIM8, ENABLE);
}
/**
  * @brief  初始化高级控制定时器
  * @param  无
  * @retval 无
  */
void TIM1_Configuration(void)
{  
	TIM1_GPIO_Config();
  TIM1_Mode_Config();
}


void TIM8_Configuration(void)
{  
	TIM8_GPIO_Config();
  TIM8_Mode_Config();
}
/*********************************************END OF FILE**********************/
