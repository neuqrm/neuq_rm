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
	if(CHASSIS_BREAK_EN)          
	{
		if(stop_flag_chassis==1)										
		apid_chassis_realize(a_chassis_p,a_chassis_i,a_chassis_d);			
	}
	chassis_speed_control(Liner_X, Liner_Y, Angular_Z);	
	
	if(TRIGGER_POS_EN)	
		trigger_angle_control(Trigger_Angle);//mhp111
	else
		trigger_control(Trigger_Speed);
    if(gimbal_modes == gimbal_can_mode)	          						
	{
		if(GIMBAL_POS_EN)                             					
			gimbal_angle_control(Angle_Yaw,Angle_Pitch);  				
		else
			gimbal_speed_control(Angular_Yaw, Angular_Pitch);     	
	}
   	vpid_chassis_realize(v_chassis_p,v_chassis_i,v_chassis_d);			
	
	if(GIMBAL_POS_EN)
		apid_gimbal_realize(a_yaw_p,a_yaw_i,a_yaw_d,a_pitch_p,a_pitch_i,a_pitch_d);	
	else
		vpid_gimbal_realize(v_yaw_p,v_yaw_i,v_yaw_d,v_pitch_p,v_pitch_i,v_pitch_d);     
	
	if(TRIGGER_POS_EN)
		apid_trigeer_realize(a_trigger_p,a_trigger_i,a_trigger_d);	//mhp
	vpid_trigger_realize(v_trigger_p,v_trigger_i,v_trigger_d);
	

	set_chassis_current();		
	set_trigger_current();
	set_gimbal_current();


	if(gimbal_modes == gimbal_pwm_mode)
	{
		TIM_SetCompare1(TIM1,pwm_pulse_p);                           
		TIM_SetCompare2(TIM1,pwm_pulse_y);                           
	}

//	fric1_on(FRIC_Speed);开环，子弹打不准                                   		
//	fric2_on(FRIC_Speed);开环，子弹打不准
	
	fric_speed_control(fric1_speed,fric2_speed);//mhp111,得到摩擦轮的目标速度
	vpid_fric_realize(v_fric1_p,v_fric1_i,v_fric1_d,v_fric2_p,v_fric2_i,v_fric2_d);//mhp111得到摩擦轮的PID_OUT
	set_fric_current();//PID_OUT输出给占空比进行pwm输出
	
	
	
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

