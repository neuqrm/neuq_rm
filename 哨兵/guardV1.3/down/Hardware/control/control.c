/**
	用于接收上位机的标志位而直接执行相关任务
**/

#include "control.h"


//全局变量
int control_flag = 0;  //控制指令标志位，需要外部声明,再调用时不用声明

void cruise_mode(void)
{
	static int yaw_flag = 0;
	static int pitch_flag = 0;
	if(control_flag == cruise)  
	{
		gimbal_loop  = speed_loop;	//切换速度环
		/********************yaw***************/
		if(Kinematics.yaw.actual_angle > 70&&Kinematics.yaw.actual_angle <80)  //左边界
			yaw_flag = 0;
		else if(Kinematics.yaw.actual_angle < 90&&Kinematics.yaw.actual_angle > 80)//右边界
			yaw_flag = 1;
		/**********************pitch**************************/
		if((Kinematics.pitch.actual_angle > 345&&Kinematics.pitch.actual_angle < 355) || (Kinematics.pitch.actual_angle < 345 && Kinematics.pitch.actual_angle > 270) ) //上边界
			pitch_flag =0;
		if((Kinematics.pitch.actual_angle < 60&&Kinematics.pitch.actual_angle > 50 )|| (Kinematics.pitch.actual_angle > 60 && Kinematics.pitch.actual_angle < 90 )) //下边界
			pitch_flag =1;
		//通过下面的方法可以减小电机误差的影响
		if(yaw_flag == 0)
			Kinematics.yaw.target_angular = cruise_left_speed;
		else if(yaw_flag == 1)
			Kinematics.yaw.target_angular = cruise_right_speed;
		if(pitch_flag == 0)
			Kinematics.pitch.target_angular = cruise_left_speed_1;
		if(pitch_flag == 1)
			Kinematics.pitch.target_angular = cruise_right_speed_1;

	}		
}
