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
	//if(off_line_flag==1)   //整车脱力状态		
		
	/*------ 运动控制函数 ------*/
	chassic_speed_control(Liner_X, Liner_Y, Angular_Z);	
	
	if(HANDLE_POS_EN)	
		handle_angle_control(Handle_Left_Angel,Handle_Right_Angle);
	else
		handle_speed_control(Handle_Left_Angular,Handle_Right_Angular);
	
	//void stepper_motor_control(int Distance);//这个是步进电机最终运动控制函数
	
	//stepper_motor_control(Stepper_Left_Angle,Stepper_Right_Angle);
	
	/*------ pid计算 ------*/	
	vpid_chassic_realize(v_chassic_p,v_chassic_i,v_chassic_d);		
	if(HANDLE_POS_EN)
	apid_handle_realize(a_handle_p,a_handle_i,a_handle_d);
	vpid_handle_realize(v_handle_p,v_handle_i,v_handle_d);
  /*------ 电流赋值 ------*/	
	set_chassis_current();	
	set_handle_current();
	
	//handle角度控制
	
	switch(handle_angle_flag)
	{
		case 0:
			handle_90();//夹爪垂直
			break;
		case 1:
			handle_180();//夹爪水平朝前
			break;
		default:
			Kinematics.handle_L.target_angle=5;
			Kinematics.handle_R.target_angle=5;
			break;
	}
	
	/*------ 舵机脉宽 ------*/
	TIM_SetCompare1(TIM2,Rescue_Claw_Pulse_Right);
	TIM_SetCompare2(TIM2,Rescue_Claw_Pulse_Left);
//	TIM_SetCompare3(TIM2,Kinematics.rescue_claw_pulse);
//	TIM_SetCompare4 (TIM2,Kinematics.rescue_claw_pulse);

	/*------ 电磁阀控制 ------*/
  switch(Solenoid_valve_flag)
	{
		case 0:
			power_close_motor(3);
			break;
		case 1:
			power_open_motor(3);
			break;
		default:break;
	}
	  switch(Solenoid_handle_flag)
	{
		case 0:
			power_close_motor(4);
			break;
		case 1:
			power_open_motor(4);
			break;
		default:break;
	}

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
		
	}
	else if(!key_press())
		key_flag=unpressed;		//按键未被按下
}

