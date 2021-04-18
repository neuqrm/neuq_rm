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

  ...........������δ�ɹ���ͬ־����Ŭ��...........
*/  
#include "My_Init.h"
/**
  *@brief  ��������ʼ��������ѭ���ȴ��ж�
  */
int main()
{
	mode_init();    
 	All_Init();			
	pid_init();        
//	TIM_SetCompare3(TIM2, 150);
	while(1)                          
	{
     		LED0=!LED0;
	}
}
/**motor5.actual_angle
  * @brief  ��ʱ��3�жϷ�����			
  */
void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{ 
		time_count++;
		if(control_mode == DJi_Remote_Control) 
			Remote_Control();				
		if(control_mode == auto_control)
		{
			if(flag_command_recieved == 1)	
			{
				resolve_json_chassis_command();
				flag_command_recieved = 0;
			}
   
			if(flag_command_recieved1 == 1)
			{
				resolve_json_gimbal_angle_command();  
				flag_command_recieved1 = 0;	
			}
	
			if(flag_command_recieved2 == 1)
			{
				resolve_json_trigger_command();  
				resolve_json_fric_command();  
				flag_command_recieved2 = 0;	
			}
			if(flag_command_recieved3 == 1)  
			{
				resolve_json_gimbal_speed_command();
				caclulate_pwm_pulse();		
				flag_command_recieved3 = 0;
			}
			if(flag_command_recieved4 == 1)	
			{
				flag_command_recieved4 = 0;	
			}
			if(flag_command_recieved5 == 1)	
			{
				resolve_json_gimbal_speed_command();
				flag_command_recieved5 = 0;	
			}
			if(flag_command_recieved6 == 1)	
			{
				resolve_json_trigger_shoot_command();
				flag_command_recieved6 = 0;	
			}
			if(flag_command_recieved7 == 1)	
			{
				resolve_json_steering_engine_command();
				flag_command_recieved7 = 0;	
			}
		}
		if(time_count%7 ==0)
		{		
			fric_record_control();  //mhp111,�õ�Ħ����ʵ���ٶ�
			Robo_Move();
		}
		if(time_count%31 == 0)		
			Debug_Key();
		if(time_count%500 == 0)		
		{
			LED1=!LED1;	
			time_count = 0;
		}
		if(time_count%4 == 0)
		{		
			Get_Base_Velocities();		
			Get_Gimbal_Angle();       
		}	
		if(MSG_SEND_EN)
		{
			if(time_count%20 == 0)			
			{   
				send_chassis_info_by_json();
				send_gimbal_info_by_json();
				send_infantry_info_by_json();
			} 		   
		}                          
		if(time_count>=1000)			
			time_count=1;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  
}

