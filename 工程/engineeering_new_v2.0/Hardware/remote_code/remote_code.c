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

#include "kinematic.h"
#include "remote_code.h"
#include "DJi_remote.h"
#include "motor.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "delay.h"
#include "flag.h"
#include "stepper_motor.h"
#include "bsp_dbus.h"
//�ڲ���������
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);

uint16_t pulse_cnt = 0;   //��������������
uint16_t i = 0;           //ץȡ��ʯ���̲����
/**
  * @brief  ң�ش��룬��ң���������Ӧ�������˾��嶯���ϣ����ڶ�ʱ���ﲻ�ϵ�ˢ
  */
void Remote_Control()    //���������Ͳ��ϵ��ж�ÿ��ͨ����ֵ�������������������Ӧ����
{	
	
	if(!LEFT_LEVER==0)			
	{
		off_line_flag=0;    //�����ѻ���ʶ��
		
		if(LEFT_LEVER == keyboard_mode)  //���̿���ģʽ
		{
			if(key_board&ws_key)
	     {
				if((key_board&W_key) == W_key)
					Liner_X = max_base_linear_speed;
				if((key_board&S_key) == S_key)
					Liner_X = -max_base_linear_speed;
			 }
			else
				Liner_X = 0;
			if(key_board&ad_key)
			{
				if((key_board&A_key) == A_key)
					Angular_Z = -max_base_rotational_speed;
				if((key_board&D_key) == D_key)
					Angular_Z = max_base_rotational_speed;	
			}
			else
				Angular_Z = 0;
			if(key_board&qe_key)
				{
				if((key_board&Q_key) == Q_key)
					Liner_Y = -max_base_linear_speed;
				if((key_board&E_key) == E_key)
					Liner_Y = max_base_linear_speed;	
				}
			else
				Liner_Y = 0;
/***** ����ģʽ *****/
			if((key_board&SHIFT_key) == SHIFT_key)
			{
				max_base_linear_speed = MAX_BASE_LINEAR_SPEED;
				max_base_rotational_speed = MAX_BASE_ROTATIONAL_SPEED;
			}
			else 
			{
				max_base_linear_speed = NORMAL_LINEAR_SPEED;
				max_base_rotational_speed = NORMAL_ROTATIONAL_SPEED;
			}
		}
		
		if(LEFT_LEVER == catch_mineral_up | LEFT_LEVER == catch_mineral_down)  //�󲦸��л�����
		{  
			Liner_X   = caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
			Liner_Y   = caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			Angular_Z = caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value, r_max_value); 
			
		  if(LEFT_LEVER == catch_mineral_up)  //�󲦸���
			{
				if(i==0)
				{
					if(RIGHT_LEVER==3)		
						stepper_motor_off();
					if(RIGHT_LEVER==1)
					{
						stepper_motor_up();
						if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
							pulse_cnt++;
					}
					if(pulse_cnt>=400)
					{
						i++;
            stepper_motor_off();
					}
				}
				else 
				{
				 if(i<=3)
				 {
					if(RIGHT_LEVER==3 && lever_flag==0)
						lever_flag=1;
					if(RIGHT_LEVER==1 && lever_flag==1)
					{
					  i++;
						lever_flag=0;
					}
				 }
					if(i==1)
					Solenoid_valve_flag=1;
					if(i==2)
					{
						Kinematics.handle_L.target_angle=8191*10;
					  Kinematics.handle_R.target_angle=8191*10;
					}
          if(i==3)		
					Solenoid_handle_flag=1;
					if(i==4)
					{
						Kinematics.handle_L.target_angle=0;
					  Kinematics.handle_R.target_angle=0;
					}
				}
			}
		  if(LEFT_LEVER == catch_mineral_down) //�󲦸���
			{
				if(i==0)
				{
					if(RIGHT_LEVER==3)		
						stepper_motor_off();
					if(RIGHT_LEVER==1 && pulse_cnt>0)
					{
						stepper_motor_down();
						if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
							pulse_cnt--;
					}
					if(pulse_cnt<=0)
					{
            stepper_motor_off();
					}
				}
				else
				{
				 if(i>0)
				 {
					if(RIGHT_LEVER==3 && lever_flag==0)
						lever_flag=1;
					if(RIGHT_LEVER==1 && lever_flag==1)
					{
					  i--;
						lever_flag=0;
					}
				 }
				  if(i==0)
						Solenoid_valve_flag=0;
					if(i==1)
					{
						Kinematics.handle_L.target_angle=0;
					  Kinematics.handle_R.target_angle=0;
					}
					if(i==2)
					  Solenoid_handle_flag=0;
          if(i==3)		
					{
						Kinematics.handle_L.target_angle=8191*10;
					  Kinematics.handle_R.target_angle=8191*10;					
					}	
					if(i==4)
					{
					}
				}
			}
		}
		if(RIGHT_LEVER == 2)    //�Ҳ�����
		{
		
		}
	}
	else
		off_line_flag=1;	
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

