/**
	���ڽ�����λ���ı�־λ��ֱ��ִ���������
**/

#include "control.h"


//ȫ�ֱ���
int control_flag = 0;  //����ָ���־λ����Ҫ�ⲿ����,�ٵ���ʱ��������

void cruise_mode(void)
{
	static int yaw_flag = 0;
	static int pitch_flag = 0;
	if(control_flag == cruise)  
	{
		gimbal_loop  = speed_loop;	//�л��ٶȻ�
		/********************yaw***************/
		if(Kinematics.yaw.actual_angle > 70&&Kinematics.yaw.actual_angle <80)  //��߽�
			yaw_flag = 0;
		else if(Kinematics.yaw.actual_angle < 90&&Kinematics.yaw.actual_angle > 80)//�ұ߽�
			yaw_flag = 1;
		/**********************pitch**************************/
		if((Kinematics.pitch.actual_angle > 345&&Kinematics.pitch.actual_angle < 355) || (Kinematics.pitch.actual_angle < 345 && Kinematics.pitch.actual_angle > 270) ) //�ϱ߽�
			pitch_flag =0;
		if((Kinematics.pitch.actual_angle < 60&&Kinematics.pitch.actual_angle > 50 )|| (Kinematics.pitch.actual_angle > 60 && Kinematics.pitch.actual_angle < 90 )) //�±߽�
			pitch_flag =1;
		//ͨ������ķ������Լ�С�������Ӱ��
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
