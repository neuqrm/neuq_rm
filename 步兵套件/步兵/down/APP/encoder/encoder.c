/**
  ******************************************************************************
  * @file    Project/APP/encoder.c 
  * @author  Zixuan Li
  * @version V1.0.0
  * @date    3.2021
  * @brief   ��������ȡĦ�����ٶ�
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "encoder.h"
#include "stm32f4xx.h"

/**

  ��������
	ֱ�Ӷ�ȡ�ߵ͵�ƽ����
	
	ʹ�ô���A4 A5
**/

void light_encoder_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOFʱ��

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//�ж������Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//�ж������Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
}

u16 ENCODER_read_left=0;//��ȡ���Ĺ��Ƶ

int last_mark_left=1; //����ϴε�Ƶ

void get_light_encoder_read_left(void)
{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1)//�ߵ�ƽ
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


u16 ENCODER_read_right=0;//��ȡ���Ĺ��Ƶ

int last_mark_right=1; //����ϴε�Ƶ

void get_light_encoder_read_right(void)
{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==1)//�ߵ�ƽ
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

//ת��������
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
    ����������
**/

int Encoder4_Timer_Overflow=0,Encoder2_Timer_Overflow=0;                                      //���������������ÿ266*4���һ�Σ�
u16 Previous_Count=0;          //�ϴ�TIM3->CNT��ֵ

//��ʱ��5ͨ��1���벶������
//arr���Զ���װֵ(TIM2,TIM5��32λ��!!)     4,294,967,296
//psc��ʱ��Ԥ��Ƶ��

void Encoder_Light_TIM4(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_5;          //GPIOA6��GPIOA7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //�ٶ�100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //����	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  //���츴�����
  GPIO_Init(GPIOA, &GPIO_InitStructure);                          //��ʼ��PA6��PA7

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_TIM4);           //GPIOA6����Ϊ��ʱ��3ͨ��1
GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_TIM4);           //GPIOA7����Ϊ��ʱ��3ͨ��2
	
  TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(����������-1)*4	�ı�Ƶԭ��
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //ʱ�ӷ�Ƶ���ӣ�����Ƶ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);                  //��ʼ��TIM3
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //ѡ�������IC1ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter =0;                            //���������˲���
  TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //ѡ�������IC2ӳ�䵽TI2��
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI2��
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter=0;                             //���������˲���
  TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );//���������ã���ʱ��������ģʽ�������ء������أ�
		
  NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;                   //��ʱ��3�жϷ�������
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //��Ӧ���ȼ�2
	NVIC_Init(&NVIC_InitStructure);                                 //���ö�ʱ��3
		
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);                        //����ʱ��3�����ж�
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
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1| GPIO_Pin_2;          //GPIOA6��GPIOA7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //�ٶ�100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //����	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  //���츴�����
  GPIO_Init(GPIOA, &GPIO_InitStructure);                          //��ʼ��PA6��PA7

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_TIM2);           //GPIOA6����Ϊ��ʱ��3ͨ��1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_TIM2);           //GPIOA7����Ϊ��ʱ��3ͨ��2
	
  TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(����������-1)*4	�ı�Ƶԭ��
  TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //ʱ�ӷ�Ƶ���ӣ�����Ƶ
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);                  //��ʼ��TIM3
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //ѡ�������IC1ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter =0;                            //���������˲���
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //ѡ�������IC2ӳ�䵽TI2��
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI2��
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter=0;                             //���������˲���
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
  TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );//���������ã���ʱ��������ģʽ�������ء������أ�
		
  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                   //��ʱ��3�жϷ�������
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //ʹ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //��Ӧ���ȼ�2
  NVIC_Init(&NVIC_InitStructure);                                 //���ö�ʱ��3
		
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                        //����ʱ��3�����ж�
  TIM_Cmd(TIM2,ENABLE);   
}

void encoder_hall_Init(void)
{
	Encoder_Light_TIM2((266-1)*4,0);
	Encoder_Light_TIM4((266-1)*4,0);
}


void TIM4_IRQHandler(void)
{
		if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)                    //����ж�
	{   
		Encoder4_Timer_Overflow++;     		
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  

}

void TIM2_IRQHandler(void)
{
		if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)                    //����ж�
	{   
		Encoder2_Timer_Overflow++;     		
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  

}

u32 Read_Encoder(int TIMx)//��ȡ�������ó��ٶ�
{
  u32 Count;                                                      //һ��ʱ����ת����������
  u16 Current_Count;                                              //��ǰTIM3->CNT��ֵ
	u16 Enc_Timer_Overflow_one;	                                   

	switch(TIMx)
	{
	case 2:
		 Enc_Timer_Overflow_one=Encoder2_Timer_Overflow;                  
     Current_Count = TIM_GetCounter(TIM2);      //��ȡTIM2_>cnt��ֵ
		 Encoder2_Timer_Overflow=0;   
     if((TIM2->CR1&0x0010) == 0x0010)                                //�����ת
     Count = (u32)((-1*Enc_Timer_Overflow_one)*(4*ENCODER_PPR-4) + (Current_Count - Previous_Count));   //�����һ��ʱ��ת����������
	   else                                                            //�����ת
		 Count = (u32)(Current_Count - Previous_Count + (Enc_Timer_Overflow_one) * (4*ENCODER_PPR-4));       //�����һ��ʱ��ת����������
     Previous_Count = Current_Count;  
     break; 
		 
	case 4:
		 Enc_Timer_Overflow_one=Encoder4_Timer_Overflow;                  
     Current_Count = TIM_GetCounter(TIM4);      //��ȡTIM4_>cnt��ֵ
		 Encoder4_Timer_Overflow=0;   
     if((TIM4->CR1&0x0010) == 0x0010)                                //�����ת
     Count = (u32)((-1*Enc_Timer_Overflow_one)*(4*ENCODER_PPR-4) + (Current_Count - Previous_Count));   //�����һ��ʱ��ת����������
	   else                                                            //�����ת
		 Count = (u32)(Current_Count - Previous_Count + (Enc_Timer_Overflow_one) * (4*ENCODER_PPR-4));       //�����һ��ʱ��ת����������
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
		speed=get_light_speed(2);//������
	}
	else speed=Read_Encoder(2);
	return speed;
	
}

u32 get_right_encoder_speed(void)
{
	u32 speed;
	if(ENCODER_EN==1) 
	{
		speed=get_light_speed(4);//������
	}
	else speed=Read_Encoder(4);
	return speed;
}
