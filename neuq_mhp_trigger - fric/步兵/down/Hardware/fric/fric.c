#include "fric.h"
#include <math.h>
#include "stm32f4xx.h"
#include "kinematic.h"
#include "motor.h"

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

    TIM_TimeBaseInitStructure.TIM_Period = 2500 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 180 - 1;//待修改
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
    //TIM_OCInitStructure.TIM_Pulse = 1000;//修改

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
//***********************mhp111
float fric1_pulse=0;
float fric2_pulse=0;
void set_fric_current(void)
{
	fric1_pulse=fric1.vpid.PID_OUT;
	fric2_pulse=fric2.vpid.PID_OUT;
	TIM_SetCompare3(TIM1, fric1_pulse);
	TIM_SetCompare4(TIM1, fric1_pulse);
}//mhp222
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

void auto_fire(void)
{
if(Kinematics.fric.target_angular==1)//自动射击使用
		{   fric1_on(1500);
				fric2_on(1500);
			  
			  trigger_control(150);
			if(motor5.actual_speed<20&&motor5.actual_speed>-20)    						//堵转
					{ 
						static int count_=1;
					  count_++;
						int   a;
						a =pow(-1,count_)*50;
						trigger_control(a);
						if(count_>100)
							count_=1;
					}
		}
		else if(Kinematics.fric.target_angular==0)
		{   
			  fric1_on(1000);
				fric2_on(1000);
			  trigger_control(0);
		}


}
//************************************************mhp111
static void TIM6_config()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 9000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 1000-1;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM6, ENABLE);
}

static void TIM7_config()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 9000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 1000-1;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM7, ENABLE);
}

void Base_TIM_config()
{
	TIM6_config();
	TIM7_config();
}

void fric_gpio_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//*********************************mhp111
int i=0;
int j=0;
int time_record1[]={0};
float time_average1=0;
int time_total1=0;
int time_record2[]={0};
float time_average2=0;
int time_total2=0;
//*********************************mhp111
static void fric1_actual_speed_record()
{
	TIM6->CNT=0;
	do{
	}while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0);
	time_record1[i]=TIM_GetCounter(TIM6);
	if(i<1000)
		i++;
	for(j=0;j<i;j++)
		time_total1+=time_record1[j];
	time_average1=time_total1/i;   //time_average1=TIM6->CNT,1个CNT=1/90ms
	fric1.vpid.actual_speed=0.5/time_average1*1000*60;   //  单位:r/min
	if(i==1000)
		i=0;
}

static void fric2_actual_speed_record()
{
	TIM7->CNT=0;
	do{
	}while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==0);
	time_record2[i]=TIM_GetCounter(TIM6);
	if(i<1000)
		i++;
	for(j=0;j<i;j++)
		time_total2+=time_record2[j];
	time_average2=time_total2/i;   //time_average2=TIM7->CNT,1个CNT=1/90ms
	fric2.vpid.actual_speed=0.5/time_average2*1000*60;   //  单位:r/min
	if(i==1000)
		i=0;
}

void fric_actual_speed_record()
{
	fric1_actual_speed_record();
	fric2_actual_speed_record();
}

void fric_record_control()
{
	Base_TIM_config();
	fric_gpio_config();
}
