/**
  ******************************************************************************
  * @file    Project/APP/encoder.c 
  * @author  Zixuan Li
  * @version V1.0.0
  * @date    3.2021
  * @brief   编码器读取摩擦轮速度
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "encoder.h"
#include "stm32f4xx.h"

/**

  光电编码器
	直接读取高低电平计数
	
	使用串口A4 A5
**/

void light_encoder_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOF时钟

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//中断输入对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//中断输入对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
}

u16 ENCODER_read_left=0;//读取到的光电频

int last_mark_left=1; //标记上次电频

void get_light_encoder_read_left(void)
{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1)//高电平
		{
			if(last_mark_left==0) 
			{
				ENCODER_read_left++;
				last_mark_left=1;
			}
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0)
		{
			if(last_mark_left==1) 
			{
				ENCODER_read_left++;
				last_mark_left=0;
			}
		}
}


u16 ENCODER_read_right=0;//读取到的光电频

int last_mark_right=1; //标记上次电频

void get_light_encoder_read_right(void)
{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==1)//高电平
		{
			if(last_mark_right==0) 
			{
				ENCODER_read_right++;
				last_mark_right=1;
			}
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==0)
		{
			if(last_mark_right==1) 
			{
				ENCODER_read_right++;
				last_mark_right=0;
			}
		}
}

//转换成速率
u32 get_light_speed(int TIMx)
{
	u16 speed;
	switch(TIMx)
	{
		 case 2:
	   speed=ENCODER_read_left/2;
	   ENCODER_read_left=0;
	   break;
		 
		 case 4:
     speed=ENCODER_read_right/2;
	   ENCODER_read_right=0;
     break;		 
		 default:break;
	}
			return speed;	

}

/**
    霍尔编码器
**/

int Encoder4_Timer_Overflow=0,Encoder2_Timer_Overflow=0;                                      //编码器溢出次数（每266*4溢出一次）
u16 Previous_Count=0;          //上次TIM3->CNT的值

//定时器5通道1输入捕获配置
//arr：自动重装值(TIM2,TIM5是32位的!!)     4,294,967,296
//psc：时钟预分频数

void Encoder_Light_TIM4(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_5;          //GPIOA6和GPIOA7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //复用模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //速度100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //浮空	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  //推挽复用输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);                          //初始化PA6和PA7

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_TIM4);           //GPIOA6复用为定时器3通道1
GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_TIM4);           //GPIOA7复用为定时器3通道2
	
  TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(编码器线数-1)*4	四倍频原理
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //时钟分频因子，不分频
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);                  //初始化TIM3
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //选择输入端IC1映射到TI1上
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI1上
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter =0;                            //配置输入滤波器
  TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //选择输入端IC2映射到TI2上
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI2上
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter=0;                             //配置输入滤波器
  TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );//编码器配置（定时器、编码模式、上升沿、上升沿）
		
  NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;                   //定时器3中断分组配置
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //响应优先级2
	NVIC_Init(&NVIC_InitStructure);                                 //配置定时器3
		
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);                        //允许定时器3更新中断
	TIM_Cmd(TIM4,ENABLE);   
}

void Encoder_Light_TIM2(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1| GPIO_Pin_2;          //GPIOA6和GPIOA7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //复用模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //速度100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //浮空	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  //推挽复用输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);                          //初始化PA6和PA7

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_TIM2);           //GPIOA6复用为定时器3通道1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_TIM2);           //GPIOA7复用为定时器3通道2
	
  TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(编码器线数-1)*4	四倍频原理
  TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //定时器分频
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //向上计数模式
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //时钟分频因子，不分频
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);                  //初始化TIM3
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //选择输入端IC1映射到TI1上
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI1上
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter =0;                            //配置输入滤波器
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //选择输入端IC2映射到TI2上
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI2上
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter=0;                             //配置输入滤波器
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
  TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );//编码器配置（定时器、编码模式、上升沿、上升沿）
		
  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                   //定时器3中断分组配置
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //使能
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //响应优先级2
  NVIC_Init(&NVIC_InitStructure);                                 //配置定时器3
		
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                        //允许定时器3更新中断
  TIM_Cmd(TIM2,ENABLE);   
}

void encoder_hall_Init(void)
{
	Encoder_Light_TIM2((266-1)*4,0);
	Encoder_Light_TIM4((266-1)*4,0);
}


void TIM4_IRQHandler(void)
{
		if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)                    //溢出中断
	{   
		Encoder4_Timer_Overflow++;     		
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  

}

void TIM2_IRQHandler(void)
{
		if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)                    //溢出中断
	{   
		Encoder2_Timer_Overflow++;     		
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  

}

u32 Read_Encoder(int TIMx)//读取编码器得出速度
{
  u32 Count;                                                      //一段时间内转过的脉冲数
  u16 Current_Count;                                              //当前TIM3->CNT的值
	u16 Enc_Timer_Overflow_one;	                                   

	switch(TIMx)
	{
	case 2:
		 Enc_Timer_Overflow_one=Encoder2_Timer_Overflow;                  
     Current_Count = TIM_GetCounter(TIM2);      //获取TIM2_>cnt的值
		 Encoder2_Timer_Overflow=0;   
     if((TIM2->CR1&0x0010) == 0x0010)                                //如果反转
     Count = (u32)((-1*Enc_Timer_Overflow_one)*(4*ENCODER_PPR-4) + (Current_Count - Previous_Count));   //计算出一个时间转过的脉冲数
	   else                                                            //如果正转
		 Count = (u32)(Current_Count - Previous_Count + (Enc_Timer_Overflow_one) * (4*ENCODER_PPR-4));       //计算出一个时间转过的脉冲数
     Previous_Count = Current_Count;  
     break; 
		 
	case 4:
		 Enc_Timer_Overflow_one=Encoder4_Timer_Overflow;                  
     Current_Count = TIM_GetCounter(TIM4);      //获取TIM4_>cnt的值
		 Encoder4_Timer_Overflow=0;   
     if((TIM4->CR1&0x0010) == 0x0010)                                //如果反转
     Count = (u32)((-1*Enc_Timer_Overflow_one)*(4*ENCODER_PPR-4) + (Current_Count - Previous_Count));   //计算出一个时间转过的脉冲数
	   else                                                            //如果正转
		 Count = (u32)(Current_Count - Previous_Count + (Enc_Timer_Overflow_one) * (4*ENCODER_PPR-4));       //计算出一个时间转过的脉冲数
     Previous_Count = Current_Count;  
     break; 
	   default:break;
	}
		 return(Count);

}

void ENCODER_Init(void)
{
	if(ENCODER_EN==1) light_encoder_Init();
	else encoder_hall_Init();
}

u32 get_left_encoder_speed(void)
{
	u32 speed;
	if(ENCODER_EN==1) 
	{
		speed=get_light_speed(2);//脉冲数
	}
	else speed=Read_Encoder(2);
	return speed;
	
}

u32 get_right_encoder_speed(void)
{
	u32 speed;
	if(ENCODER_EN==1) 
	{
		speed=get_light_speed(4);//脉冲数
	}
	else speed=Read_Encoder(4);
	return speed;
}
