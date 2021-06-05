#include "steering_engine.h"
#include "stm32f4xx.h"


 void TIM_Steering_Engine_PWM_Init()  //定时器2通道1对应舵机1，频率50Hz PA0
{									 //定时器2通道2对应舵机2，频率52Hz PA1
									 //定时器2通道3对应舵机3，频率50Hz PA2
									 //定时器2通道4对应舵机4，频率52Hz PA3
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);   
    
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
      
      
    TIM_TimeBaseStructure.TIM_Period = 2000-1;  
    TIM_TimeBaseStructure.TIM_Prescaler =900-1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 0; //初始脉宽50
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
  
  
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);       

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);   

	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);       

	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2,ENABLE);
    TIM_Cmd(TIM2, ENABLE);
} 







