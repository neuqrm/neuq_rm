/**
  ******************************************************************************
  * @file    Project/HARDWARE/gimbal.c 
  * @author  Junyu Luo
  * @version V1.0.0
  * @date    11.2019
  * @brief   ��̨����
  ******************************************************************************
  * @attention
  ******************************************************************************
*/

#include "gimbal.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "sys.h"
float pwm_pulse_p=1500;   //��̨��ʼ������
float pwm_pulse_y=1290;

/**
  * @breif ��̨��ʱ����ʼ������
	*/
void TIM1_GIMBAL_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
	
  //GPIO_AF_Set(GPIOA, BIT1, AF1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOA1����Ϊ��ʱ��1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);       

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOA1����Ϊ��ʱ��1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
 
   //��ʼ��TIM1
	TIM_TimeBaseStructure.TIM_Period = 19999; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 179; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM1 Channel PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCNPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = 1500;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM1, ENABLE); 
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

