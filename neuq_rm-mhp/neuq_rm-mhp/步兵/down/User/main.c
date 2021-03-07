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
/**
  *@brief  主函数初始化，进入循环等待中断
  */
int main()
{
 	All_Init();												
	pid_init();                     
	
	while(1)                        
	{
		LED0=!LED0;
		delay_ms(500);
	}
}



void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 	
	{ 
		time_count++;
		Remote_Control();				
		if(flag_command_recieved == 1)	
		{
			if(1)
				flag_command_recieved = 0;	
		}
   
		if(flag_command_recieved1 == 1)
		{
			if(1)
				resolve_json_gimbal_speed_command();
			flag_command_recieved1 = 0;	
		}
	 
	 	if(flag_command_recieved2 == 1)
		{
			if(1)
				flag_command_recieved2 = 0;	
		}
		if(flag_command_recieved3 == 1)  
		{
			if(1)
				flag_command_recieved3 = 0;	
		}
			if(flag_command_recieved4 == 1)	
		{
			if(1)
				flag_command_recieved4 = 0;	
		}
			if(flag_command_recieved5 == 1)
		{
			if(1)
			  flag_command_recieved5 = 0;	
		}
		if(time_count%7 ==0)			
			Robo_Move();
		if(time_count%31 == 0)		
			Debug_Key();
		if(time_count%500 == 0)			
		{
			LED1=!LED1;							
			time_count = 0;
		}
		if(time_count%4 == 0)		
			Get_Base_Velocities();		
		if(time_count%20 == 0)			
			send_chassis_info_by_json();
		if(time_count>=1000)			
			time_count=1;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); 
}

