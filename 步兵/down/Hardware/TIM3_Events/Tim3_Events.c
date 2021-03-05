/**
  ******************************************************************************
  * @file    Project/USER/Tim3_Events.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   定时器3相关函数
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include "Tim3_Events.h"


/**
  * @breif 运动控制函数
	* @param  各pid参数
	*/
void Robo_Move()
{
	if(stop_flag_1 && ap_pid_flag == ang_pid)			//如果此时速度为0（停止）或者自动控制时没有在运行位置闭环   那么角度闭环
	{	
		break_jugement();
		if(stop_flag_3 && 1)
		{
			stop_chassis_motor();
		}
		apid_chassic_realize(0.2,0.05,0);			
	
	}
  if(1) //(Control_Mode) == 0x03 Control_Mode & auto_control) == auto_control)
	{
		chassic_speed_control(Liner_X, Liner_Y, Angular_Z);		 
		gimbal_speed_control(Angular_Yaw, Angular_Pitch);      //使用云台速度环控制
	  //gimbal_angle_control(Angle_Yaw,Angle_Pitch);  //使用云台位置环控制
		//auto_fire();//上位机发送标志位  开火
	 }
 
	
   	vpid_chassic_realize(v_chassic_p,v_chassic_i,v_chassic_d);			//速度闭环2  0.05
	  vpid_trigger_realize(v_trigger_p,v_trigger_i,v_trigger_d);      //拨弹轮速度闭环  参数未确定   2.5  0.05
    //apid_gimbal_realize(a_yaw_p,a_yaw_i,a_yaw_d,a_pitch_p,a_pitch_i,a_pitch_d);	        //云台位置环给定
	  vpid_gimbal_realize(v_yaw_p,v_yaw_i,v_yaw_d,v_pitch_p,v_pitch_i,v_pitch_d);     //  云台速度环设定
		set_chassis_current();		//设定电机电流
	  set_trigger_current();
	  set_gimbal_current();

	  TIM_SetCompare1(TIM1,pwm_pulse_p);
		TIM_SetCompare2(TIM1,pwm_pulse_y);

	  
}



/**
  * @breif 调试用的按键，即单片机上的白色按键
	* @param key_flag
	*/
void Debug_Key()
{
	static int key_flag = unpressed;		//用于控制下面判断在按下的过程中只进入一次
	if( key_press() && key_flag == unpressed)		//如果按键被按下
	{
		LED4=!LED4;												//LED4反转，用于状态指示
		key_flag = pressed;			          //按键被按下
		// your codes below
		
		//NRF24L01_TxPacket(send_data);			//nrf发送字符串
	}
	else if(!key_press())
		key_flag=unpressed;		//按键未被按下
}

