/**
  ******************************************************************************
  * @file    Project/USER/main.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................

  ...........革命尚未成功，同志仍需努力...........
*/  
#include "My_Init.h"

extern uint16_t pulse_cnt;   //步进电机脉冲计数
extern int a;
/**
  *@brief  主函数初始化，进入循环等待中断
  */
int main()
{
	//software_init(); //软件及结构体初始化
	//hardware_init(); //硬件初始化
	All_Init();
	//TIM_SetCompare3(TIM2, 150);
	while(1)                        
	{
		
	}
}

/**
  * @brief  定时器3中断服务函数			1ms
  */
void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 	//溢出中断
	{ 
		  time_count++;
		
		/*****   遥控器控制    ******/
		  Remote_Control();				//遥控器控制代码		
			
		/****  机器人运动控制  *****/
		if(time_count%7 ==0)		//7ms
			Robo_Move();
		/*****    按键扫描   ******/
		if(time_count%31 == 0)		//31ms  消抖
			Debug_Key();
		/*****    工作状态指示灯   ******/
		if(time_count%500 == 0)			//500ms
		{
			LED1=!LED1;							//单片机正常工作状态指示灯
			time_count = 0;
		}
		if(time_count%4 == 0)		//4ms  测速
			Get_Base_Velocities();		//计算底盘中心实际速度
		   
		if(time_count>=1000)			//清除计数标志    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}



void TIM4_IRQHandler(void)
{
	if(a==0)
	{
		
		if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 	//溢出中断
			pulse_cnt++;
	}
	if(a==1)
	{
		if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 	//溢出中断
			pulse_cnt--;
	}
	
	
	

    limit_switch.count++;
    if(limit_switch.count == stepper_motor_left.pwm_num)
    {
		limit_switch.count = 0;
		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
    }

	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
}
