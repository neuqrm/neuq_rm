/*#include "buzzer.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "delay.h"


//TIM12 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM12_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//ʹ��PORTHʱ��	
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource6,GPIO_AF_TIM12); //GPIOH6����Ϊ��ʱ��12
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOH6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //��ʼ��PH6
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);//��ʼ����ʱ��12
	
	//��ʼ��TIM12 Channel PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM12OC1

	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM12,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM12, ENABLE);  //ʹ��TIM12								  
}  


// ����: nrf_no_exist()
// ����: �������nrf�Ƿ���ڵ�Ƭ���ϣ����û�У������������������
// �������Ƿ�ʹ��
// �������
void nrf_no_exist(int able)
{
	TIM_Cmd(TIM12, ENABLE); 
	if(able==1)
	{
		TIM_SetCompare1(TIM12,100);	//�޸ıȽ�ֵ���޸�ռ�ձȣ���Χ0--1000
		delay_ms(300);
		TIM_SetCompare1(TIM12,0);	//�޸ıȽ�ֵ���޸�ռ�ձȣ���Χ0--1000
		delay_ms(300);
	}
	else
		TIM_Cmd(TIM12, DISABLE); 
}

// ����: nrf_no_connect()
// ����: ������鵥Ƭ���ϵ�nrf�Ƿ�����Զ�nrfͨѶ�ɹ�����û�гɹ����򷢳�������
// �������Ƿ�ʹ��
// �������
void nrf_no_connect(int able)
{
	TIM_Cmd(TIM12, ENABLE); 
	if(able==1)
	{
		TIM_SetCompare1(TIM12,100);	//�޸ıȽ�ֵ���޸�ռ�ձȣ���Χ0--1000
		delay_ms(700);
		TIM_SetCompare1(TIM12,0);	//�޸ıȽ�ֵ���޸�ռ�ձȣ���Χ0--1000
		delay_ms(200);
	}
	else
		TIM_Cmd(TIM12, DISABLE); 
}
*/
