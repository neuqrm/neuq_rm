/**
  ******************************************************************************
  * @file    Project/HARDWARE/remote_code.c 
  * @author  Siyuan Qiao & Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention ң�������Ƴ���
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/

#include "remote_code.h"
#include "DJi_remote.h"
#include "motor.h"
#include "kinematic.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "delay.h"

u8 Control_Mode = control_mode;

//�ڲ�ȫ�ֱ������������
float x_speed=0,y_speed=0,r_speed=0,trigger_speed=0,theta=0;
float cx_speed=0,cy_speed=0;
int flag = 0;
//�ڲ���������
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);

/**
  * @brief  ң�ش��룬��ң���������Ӧ�������˾��嶯���ϣ����ڶ�ʱ���ﲻ�ϵ�ˢ
  */
void Remote_Control()    //���������Ͳ��ϵ��ж�ÿ��ͨ����ֵ�������������������Ӧ����
{	
	if(Remote_control_mode>remote_min_value && Remote_control_mode<remote_max_value)		//�������ң������
	{
		//��־λ��Ϊң��ģʽ
		Control_Mode &= remote_control;											//�޸�Control_Mode�ڶ�λΪ0
	}
	else
	{
		//��־λ��Ϊ�Զ�ģʽ
		Control_Mode |= auto_control;												//�޸�Control_Mode�ڶ�λΪ1		
	} 
	
	if((Control_Mode & auto_control) != auto_control)			//�������ģʽ�������Զ����ƣ���ң�ؿ���
	{
		if(Remote_control_mode == chassis_CH_width)
		{  
			x_speed=caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
			y_speed=caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			r_speed=caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value,r_max_value);  
		}						
	}
			
		if((Control_Mode&DJi_Remote_Control) == DJi_Remote_Control)
		{
			y_speed = y_speed;
			r_speed = -r_speed; //ȡ����ʹ��ʱ����תΪ����
		}
		else if((Control_Mode&FS_Remote_Control) == FS_Remote_Control)		//��ΪFS_Remote_Control = 0������ж�ʱ�������else if��
		{
			y_speed = -y_speed;
		}
		Kinematics.target_velocities.linear_x=x_speed;//����robomove��ִ��.
		Kinematics.target_velocities.linear_y=y_speed;
		Kinematics.target_velocities.angular_z=r_speed;
		trigger_control(trigger_speed);	
	}
	



// ����: caculate_speed()
// ����: ��ң����ҡ�����ӳ�䵽�����������ٶ���
// ������width��ͨ��ֵ 
//			 mid��ͨ���м�ֵ 
//			 min��ͨ�������Сֵ
//       max��ͨ��������ֵ
// �������Ӧ���ٶ�ֵ
//�ڲ��������û��������
static float caculate_linear_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_linear_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_linear_speed);
  else
    speed=0;
  return speed;                
}

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_rotational_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_rotational_speed);
  else
    speed=0;
  return speed;                
}





