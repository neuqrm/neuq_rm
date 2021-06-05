#include "stepper_motor.h"
#include "stm32f4xx.h"
#include "remote.h"
#include "motor.h"

int distance=0;
void limit_init()
{

}
void Stepper_Motor_EN_GPIO_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE,GPIO_Pin_4|GPIO_Pin_5);
}

void Stepper_Motor_DIR_GPIO_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);
}

void Stepper_Motor_VCC_GPIO_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_4|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_0);
	GPIO_SetBits(GPIOC,GPIO_Pin_4);
	GPIO_SetBits(GPIOC,GPIO_Pin_3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_1);
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
}




void TIM_Stepper_Motor_PWM_Init()
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
 
	TIM_TimeBaseStructure.TIM_Period = 10000-1; 
	TIM_TimeBaseStructure.TIM_Prescaler =150-1; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_Pulse=500;  
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 

	TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE );						 //使能指定的TIM3中断,允许更新中断

	TIM_Cmd(TIM4, ENABLE);  
}


void TIM4_NVIC_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 				//设置NVIC中断分组2:2位抢占优先级，2位响应优先级	

 	NVIC_InitTypeDef NVIC_InitStructure;							//中断优先级NVIC设置
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  				//TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  		//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  			//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  								//初始化NVIC寄存器
}

//void stepper_motor_control(int left_angle,int right_angle)
//{
//	TIM_SetCompare1(TIM4,left_angle);//还未进行换算，待修改。
//	TIM_SetCompare2(TIM4,left_angle);
//}	

void stepper_motor_up()
{
	TIM_Cmd(TIM4,ENABLE);
	//TIM_Cmd(TIM2,ENABLE);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);   //步进电机正转
	GPIO_ResetBits(GPIOE,GPIO_Pin_12);   //步进电机正转
	
    GPIO_ResetBits(GPIOE,GPIO_Pin_4);   //使能步进电机
    GPIO_ResetBits(GPIOE,GPIO_Pin_5);   //使能步进电机

}

void stepper_motor_down()
{ 
	TIM_Cmd(TIM4,ENABLE);
	//TIM_Cmd(TIM2,ENABLE);
	GPIO_ResetBits(GPIOE,GPIO_Pin_6);     //步进电机反转
	GPIO_SetBits (GPIOE,GPIO_Pin_12);     //步进电机反转
	
    GPIO_ResetBits(GPIOE,GPIO_Pin_4);   //使能步进电机
    GPIO_ResetBits(GPIOE,GPIO_Pin_5);   //使能步进电机

}

void stepper_motor_off()
{
  //GPIO_SetBits(GPIOE,GPIO_Pin_5);   //失能步进电机
  //GPIO_SetBits(GPIOE,GPIO_Pin_4);   //失能步进电机
  //TIM_Cmd(TIM2,DISABLE);
  TIM_Cmd(TIM4,DISABLE);
} 

void stepper_motor_config(int left_motor_round,int right_motor_round,int subdivision_value)//参数分别为电机圈数，电机圈数，步进电机驱动器细分数
{
	TIM_SetCompare1(TIM4,100);
	TIM_SetCompare2(TIM4,100);
	stepper_motor_left.pwm_num=360/1.8*left_motor_round*subdivision_value;
	stepper_motor_right.pwm_num=360/1.8*right_motor_round*subdivision_value;
}	

 void stepper_motor_control(int Distance)//这个是步进电机最终运动控制函数
{
	stepper_motor_left.round_cnt=Distance/k;//k为比例系数，需要测定
	stepper_motor_right.round_cnt=Distance/k;
	stepper_motor_config(stepper_motor_left.round_cnt,stepper_motor_right.round_cnt,1);
}

