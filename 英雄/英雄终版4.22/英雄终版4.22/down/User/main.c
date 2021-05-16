/**
  ******************************************************************************
  * @file    Project/USER/main.c 
  * @author  Siyuan Qiao& Junyu Luo& Zixuan Li
  * @version V1.0.0
  * @date    3.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................

  ...........革命尚未成功，同志仍需努力...........
*/  
#include "My_Init.h"
#include "stm32f4xx_it.h"
#include "encoder.h"
#include "remote.h"
#include "remote_code.h"
#include "bsp_supercap_usart.h"
#include "referee.h"
/**
  *@brief  主函数初始化，进入循环等待中断
  */
  //1591
  //6920  6000  5920
extern float	r_speed;
extern float	theta;

int main()
{	
   mode_init();                      //初始化机器人模式控制
 	All_Init();						 //机器人硬件及结构体初始化
	pid_init();                       //初始化pid各项参数的值
  motor_Init_angle();                   //初始化云台电机的初始角度
	//supercap_Init();                  //超级电容初始化
	while(1)                          //进入循环  
	{
		//use_supercap();             //使用超级电容
		LED0=!LED0;
		delay_ms(500);
	/*if(gimbal_p.actual_angle>5800)  Kinematics.yaw.target_angle=1.3;
	if(gimbal_p.actual_angle<5460)  Kinematics.yaw.target_angle=5;
			*/
		/*r_speed=0.7f;
		if(theta>0.95&&theta<5.33)  r_speed=-r_speed;	
		*/
		
	}
}

/**
  * @brief  定时器3中断服务函数			1ms
  */
int  speed_1=0,speed_2=0;
void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 	//溢出中断
	{ 
		time_count++;
		
		/*****   遥控器控制    ******/
		if(control_mode == DJi_Remote_Control) //判断是否为遥控模式
		remote_control_test();
			/****  ROS上位机控制  *****/
		if(aim_mode == auto_control) //判断是否为自动控制模式
	{
		if(flag_command_recieved == 1)	//每一毫秒检查一次是否收到控制指令，若标识符为一说明对应指令待解析，下述同理
		{
			resolve_json_chassis_command(); //底盘指令
			flag_command_recieved = 0;	
		}
   
	 if(flag_command_recieved1 == 1)
		{
			resolve_json_gimbal_angle_command();  //云台速度指令
			flag_command_recieved1 = 0;	
		}
	
	 	if(flag_command_recieved2 == 1)
		{
		  resolve_json_trigger_command();  //拨弹轮指令
      resolve_json_fric_command();     //摩擦轮指令
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
	    //resolve_json_pidparam_command(); //接受上位机pid参数调整
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
		  Get_Gimbal_Angle();       //计算云台实时角度
		  read_power();
		if(time_count%5 ==0)      //5ms 测试摩擦轮
			
		
		/****    向上位机发送数据   *****/
		if(MSG_SEND_EN)
		{
		if(time_count%20 == 0)		//20ms，50Hz 		
     {   
       send_chassis_info_by_json();
	     send_gimbal_info_by_json();
		   //send_infantry_info_by_json();
     } 		   
	  }                          
		
		if(time_count>=1000)			//清除计数标志    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
//20ms
void TIM5_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) 	//溢出中断
   {
      Get_frie_speed();
   }

   TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位

}

