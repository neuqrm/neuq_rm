#include "fric.h"
#include <math.h>
#include "stm32f4xx.h"
#include "kinematic.h"
#include "motor.h"
#include "encoder.h"
#include "Tim3_Events.h"

extern Kinematics_t Kinematics;
void fric_PWM_configuration(void) //
{

    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 20000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 180-1;//���޸�
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 1000;

    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);

    //fric_off();

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIO_SetBits(GPIOF, GPIO_Pin_10);
}

void fric_off(void)
{
    TIM_SetCompare3(TIM1, Fric_OFF);
    TIM_SetCompare4(TIM1, Fric_OFF);
}
void fric1_on(uint16_t cmd)
{
    TIM_SetCompare3(TIM1, cmd);
}
void fric2_on(uint16_t cmd)
{
    TIM_SetCompare4(TIM1, cmd);
}

/**
Ħ���ֱջ�����
**/
void fric_control_close(int fire1_speed,int fire2_speed)
{
	set_fire_speed(fire1_speed,fire2_speed);
	vpid_fire_realize(v_fire_p,v_fire_i,v_fire_d);
	if(fire1.vpid.PID_OUT<=1000) fire1.vpid.PID_OUT=1000;
	if(fire2.vpid.PID_OUT<=1000) fire2.vpid.PID_OUT=1000;
	if(fire1.vpid.PID_OUT>=2000) fire1.vpid.PID_OUT=2000;
	if(fire2.vpid.PID_OUT>=2000) fire2.vpid.PID_OUT=2000;
	fric1_on(fire1.vpid.PID_OUT);
	fric2_on(fire2.vpid.PID_OUT);
}

void auto_fire(void)
{
if(Kinematics.fric1.target_angular==1)//�Զ����ʹ��
		{   fric1_on(1500);
				fric2_on(1500);
			  
			  trigger_speed_angle(150);
			if(motor5.actual_speed<20&&motor5.actual_speed>-20)    						//��ת
					{ 
						static int count_=1;
					  count_++;
						int   a;
						a =pow(-1,count_)*50;
						trigger_speed_angle(a);
						if(count_>100)
							count_=1;
					}
		}
		else if(Kinematics.fric1.target_angular==0)
		{   
			  fric1_on(1000);
				fric2_on(1000);
			  trigger_speed_angle(0);
		}


}
