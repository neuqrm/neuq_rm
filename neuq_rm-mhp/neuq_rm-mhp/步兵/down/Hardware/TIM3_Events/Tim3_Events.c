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


void Robo_Move()
{
	if(stop_flag_1 && ap_pid_flag == ang_pid)			
	{	
		break_jugement();
		if(stop_flag_3 && 1)
			stop_chassis_motor();
		apid_chassic_realize(0.2,0.05,0);			
	}
	if(1) 
	{
		chassic_speed_control(Liner_X, Liner_Y, Angular_Z);		 
		gimbal_speed_control(Angular_Yaw, Angular_Pitch);     
	}
   	vpid_chassic_realize(v_chassic_p,v_chassic_i,v_chassic_d);			
	vpid_trigger_realize(v_trigger_p,v_trigger_i,v_trigger_d);          
	vpid_gimbal_realize(v_yaw_p,v_yaw_i,v_yaw_d,v_pitch_p,v_pitch_i,v_pitch_d);     
	set_chassis_current();		
	set_trigger_current();
	set_gimbal_current();
	TIM_SetCompare1(TIM1,pwm_pulse_p);
	TIM_SetCompare2(TIM1,pwm_pulse_y);
}



void Debug_Key()
{
	static int key_flag = unpressed;		
	if( key_press() && key_flag == unpressed)		
	{
		LED4=!LED4;												
		key_flag = pressed;			        
	}
	else if(!key_press())
		key_flag=unpressed;		
}

