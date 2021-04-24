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
int flag_1=0;
int a=10;
/**
  * @breif 运动控制函数
	* @param  各pid参数
	*/
void Robo_Move()
{
	if(stop_flag_1 && ap_pid_flag == ang_pid)			
	{	
		break_jugement();
		if(stop_flag_3 && 1)
		{
			stop_chassis_motor();
		}
		apid_chassic_realize(0.2,0.05,0);			
	}
	
	chassic_speed_control(Liner_X, Liner_Y, Angular_Z);		 
	
	//stepper_motor_control(Left_Motor_Angle,Right_Motor_Angle);

	a++;
	stepper_motor_control(a,a);
	
	
	vpid_chassic_realize(v_chassic_p,v_chassic_i,v_chassic_d);			  
	 
	set_chassis_current();		
	if(1000==a)
		a=10;
	GPIO_SetBits(GPIOE,GPIO_Pin_4 | GPIO_Pin_6);
	
	
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

