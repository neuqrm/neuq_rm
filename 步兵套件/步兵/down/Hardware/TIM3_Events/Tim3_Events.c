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
	if(CHASSIS_BREAK_EN)          //底盘刹车使能
	{
	 if(stop_flag_chassis==1)			//如果此时速度为0（停止）那么角度闭环
		apid_chassis_realize(a_chassis_p,a_chassis_i,a_chassis_d);		//底盘位置环给定	
  }
	if(steering_engine==1)
	steering_engine_on();
	if(steering_engine==0)
	steering_engine_off();
 /***** 各运动控制函数 ******/
		chassis_speed_control(Liner_X, Liner_Y, Angular_Z);		 //底盘速度控制
	if(trigger_mode == position_loop)	
		trigger_angle_control(Trigger_Angle);
	else
		trigger_control(Trigger_Speed);                      //拨弹轮速度控制
	
    if(gimbal_modes == gimbal_can_mode)	          //如果在云台模式下
		{
	    if(gimbal_loop == position_loop)                             //云台位置环使能
		   gimbal_angle_control(Angle_Yaw,Angle_Pitch);  //云台位置环控制
		  else
		   gimbal_speed_control(Angular_Yaw, Angular_Pitch);      //云台速度环控制	
	  }
		//auto_fire();//上位机发送标志位  开火
	 
 /***** pid计算及电流赋值 *****/
   	vpid_chassis_realize(v_chassis_p,v_chassis_i,v_chassis_d);			//速度闭环2  0.05
	if(trigger_mode == position_loop)
		apid_trigeer_realize(a_trigger_p,a_trigger_i,a_trigger_d);	
	vpid_trigger_realize(v_trigger_p,v_trigger_i,v_trigger_d);
		if(gimbal_loop == position_loop)
     apid_gimbal_realize(a_yaw_p,a_yaw_i,a_yaw_d,a_pitch_p,a_pitch_i,a_pitch_d);	        //云台位置环给定
		
	  vpid_gimbal_realize(v_yaw_p,v_yaw_i,v_yaw_d,v_pitch_p,v_pitch_i,v_pitch_d);     //  云台速度环设定

 /***** 设定电机电流电压值 *****/	
	  set_chassis_current();		
	  set_trigger_current();
	  set_gimbal_current();

 /***** pwm脉宽调制 *****/
		if(gimbal_modes == gimbal_pwm_mode)
		{
	     TIM_SetCompare1(TIM1,pwm_pulse_p);                              //pitch轴
		 TIM_SetCompare2(TIM1,pwm_pulse_y);                              //yaw轴
		}
		 fric1_on(FRIC_Speed);                                           //摩擦轮控制
		 fric2_on(FRIC_Speed);
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

