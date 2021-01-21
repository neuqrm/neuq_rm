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
 	All_Init();												//机器人所有配置初始化	
	while(1)                          //进入循环
	{
		LED0=!LED0;
		delay_ms(500);
	}
}


//定时器3中断服务函数			1ms
void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 	//溢出中断
	{ 
		time_count++;
		
		/*****   遥控器控制    ******/
		Remote_Control();				//遥控器控制代码
		
		/****  ROS上位机控制  *****/
		if(flag_command_recieved == 1)	//每一毫秒检查一次是否收到控制指令
		{
			//如果自动控制才可以给命令的目标速度赋值
			if(1)//(Control_Mode & auto_control) == auto_control
			resolve_json_chassis_command();
		
			flag_command_recieved = 0;	//命令接收标志位清零
		}
   
	 if(flag_command_recieved1 == 1)
		{
		if(1)//(Control_Mode & auto_control) == auto_control
			/*resolve_json_gimbal_command();
		  caclulate_pwm_pulse();	*/	
			flag_command_recieved1 = 0;	//命令接收标志位清零
		}
	 
	 	if(flag_command_recieved2 == 1)
		{
		if(1)//(Control_Mode & auto_control) == auto_control
			//resolve_json_trigger_command();
      //resolve_json_fric_command();
	
			flag_command_recieved2 = 0;	//命令接收标志位清零
		}
		if(flag_command_recieved3 == 1)  //1代表接受上位机命令，0代表不接受上位机命令
		{
		if(1)//(Control_Mode & auto_control) == auto_control
			//resolve_json_gimbal_command();
		  //caclulate_pwm_pulse();		

			
			flag_command_recieved3 = 0;	//命令接收标志位清零
		}
			if(flag_command_recieved4 == 1)	//每一毫秒检查一次是否收到控制指令
		{
			//如果自动控制才可以给命令的目标速度赋值
			if(1)//(Control_Mode & auto_control) == auto_control
	   // resolve_json_pidparam_command();
		
			flag_command_recieved4 = 0;	//命令接收标志位清零
		}
			if(flag_command_recieved5 == 1)	//每一毫秒检查一次是否收到控制指令
		{
			//如果自动控制才可以给命令的目标速度赋值
			if(1)//(Control_Mode & auto_control) == auto_control
				resolve_json_gimbal_command();
		    caclulate_pwm_pulse();		
			  flag_command_recieved5 = 0;	//命令接收标志位清零
		}
		/****  机器人运动控制  *****/
		if(time_count%7 ==0)		//7ms
		//	chassis_behavior();
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
		
		
		if(time_count%20 == 0)		//20ms，50Hz 		
  {   
        send_chassis_info_by_json();
	   //send_gimbal_info_by_json();
		 //send_infantry_info_by_json();
    //send_info_by_json();

		 //send_infantry_info_by_json();
  } 		   
		                            
		
		if(time_count>=1000)			//清除计数标志    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
void TIM4_IRQHandler(void)//周期50ms，定时器频率20HZ
{	
	
	/*static int time_count=1;

	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 	//溢出中断
	{
		
	time_count++;

	 Robo_Move();
	}*/
}

