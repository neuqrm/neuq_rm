/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �߼����ƶ�ʱ�������������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F407 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_advance_tim.h"

__IO uint16_t ChannePulse = 1000;
__IO uint16_t ChannePulseHero = 1000;
__IO uint16_t ChannePulseInfantry = 400;
uint16_t Helm_pluse = 1000;

/**
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void TIM1_GPIO_Config(void) 
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*������ʱ����ص�GPIO����ʱ��*/
	RCC_AHB1PeriphClockCmd (TIM1_PWM12_GPIO_CLOCK , ENABLE); 
	RCC_AHB1PeriphClockCmd (TIM1_PWM34_GPIO_CLOCK , ENABLE); 

	
	/* ��ʱ���������ų�ʼ�� */															   
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
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*������ʱ����ص�GPIO����ʱ��*/
	RCC_AHB1PeriphClockCmd (TIM8_PWM_GPIO_CLOCK , ENABLE); 

	
	/* ��ʱ���������ų�ʼ�� */															   
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
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У��������У�������ʱ����
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������(������ʱ��)
 * TIM_RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */
static void TIM1_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// ����TIMx_CLK,x[1,8] 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������19999����Ϊ2000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Period = 20000-1;
	// �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK=180MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 180-1;	
  // ����ʱ�ӷ�Ƶ 1��Ƶ ��180MHz
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  // ������ʽ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  // �ظ�������
  TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��TIMx, x[1,8]
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
  /*PWMģʽ����*/
	//����ΪPWMģʽ1
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
	
	// ʹ�ܶ�ʱ��
	TIM_Cmd(TIM1, ENABLE);	
	
	/* �������ʹ�� */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


static void TIM8_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// ����TIMx_CLK,x[1,8] 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); 

  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������19999����Ϊ2000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Period = 20000-1;
	// �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK=180MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 180-1;	
  // ����ʱ�ӷ�Ƶ 2��Ƶ ��90MHz
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV4;
  // ������ʽ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  // �ظ�������
  TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��TIMx, x[1,8]
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
  /*PWMģʽ����*/
	//����ΪPWMģʽ1
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
	
	// ʹ�ܶ�ʱ��
	TIM_Cmd(TIM8, ENABLE);	
	
	/* �������ʹ�� */
  TIM_CtrlPWMOutputs(TIM8, ENABLE);
}
/**
  * @brief  ��ʼ���߼����ƶ�ʱ��
  * @param  ��
  * @retval ��
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
