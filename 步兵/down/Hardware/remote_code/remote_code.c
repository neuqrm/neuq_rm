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
#include "motor.h"
#include "kinematic.h"
#include "fric.h"
#include "stm32f4xx_tim.h"
#include "gimbal.h"
#include "imuReader.h"              
#include <math.h>
#include "mode.h"
#include "delay.h"
#include "bsp_dbus.h"
#include "shoot.h"
#include "steering_engine.h"

//�ڲ�ȫ�ֱ������������
float x_speed=0,y_speed=0,r_speed=0,trigger_speed=0,trigger_angle=0,theta=0;
float cx_speed=0,cy_speed=0,fric_angular=1000;
float yaw_angular=0,pitch_angular=0;
int bullet_flag = 0;
int bullet_num=0;
int aim_flag=0;
int aim_flag_2=0;
int cabin_flag=0;
int cabin_flag_1=0;
int chassis_flag=0;
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
	
			if(Remote_control_mode == chassis_CH_width)
			{  
				 x_speed=caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
				 y_speed=caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			   r_speed=caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value,r_max_value);  
		  }
			if(Remote_control_mode == gimbal_CH_width)
			{
				switch(gimbal_modes)
			 {
				case(gimbal_pwm_mode):  				//pwmģʽ�¿�����̨ת��

			  	pwm_pulse_p=caculate_gimbal_pitch_angle(i_CH_width,i_initial_value,i_min_value,i_max_value);
				  pwm_pulse_y=caculate_gimbal_yaw_angle(x_CH_width,x_initial_value,x_min_value,x_max_value);
				
				break;
				case(gimbal_can_mode):           //canģʽ�¿�����̨ת��
					Kinematics.pitch.target_angle = caculate_gimbal_pitch_angle(i_CH_width,i_initial_value,i_min_value,i_max_value);
					Kinematics.yaw.target_angle = caculate_gimbal_yaw_angle(x_CH_width,x_initial_value,x_min_value,x_max_value);
        break;
				default:break;
        }
			}

		if(Remote_control_mode == dance_CH_width)	//С����ģʽ
		{
		     x_speed=caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
				 y_speed=caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			   r_speed=caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value,r_max_value); 
		     theta = Kinematics.actual_velocities.angular_z * 0.004f + theta; 
	       theta = yawRead();
		     cx_speed = x_speed*cos(theta) + y_speed*sin(theta);
		     cy_speed = y_speed*cos(theta) - x_speed*sin(theta);
         x_speed=cx_speed;
	       y_speed=cy_speed;
	       gimbal_speed_control(yaw_angularRead(),Kinematics.pitch.target_angular);
		     set_gimbal_current();
		}
		
		switch (trigger_control_mode)                                               //ң�����Ҳ���
				{
					case 1:
					fric_angular=1500;
          static int count_1=1;	
					count_1++;
					if(count_1>100)
					{trigger_speed = 100;
					    count_1=1;           }
					if(motor5.actual_speed<20&&motor5.actual_speed>-20)    						//��ת
					{ 
						static int count_=1;
					  count_++;
						trigger_speed =pow(-1,count_)*50;
						if(count_>100)
							count_=1;
					}
					break;
					case 2:
					trigger_speed = -50;
				  fric_angular=1000;
					if(motor5.actual_speed<20&&motor5.actual_speed>-20)    						//��ת
					{ 
						static int count_=1;
					  count_++;
						trigger_speed =pow(-1,count_)*50;
						if(count_>100)
							count_=1;
					}
					break;
					case 3:
					trigger_speed = 0;
					fric_angular=1000;
					break;
					
					default:
	      	break;
				 }											

		if((control_mode) == DJi_Remote_Control)
		{
			y_speed = y_speed;
			r_speed = -r_speed; //ȡ����ʹ��ʱ����תΪ����
			x_speed = -x_speed;
		}
			dji_remote_assignment();  //������ң�ؼ������ݸ�ֵ��ִ�в�����robomove��
  
}
/**
  * @brief  ������ϲ���ϵͳ���ƴ���
  */
void remote_control_test()
{
		if((Remote_control_mode==chassis_CH_width) || (Remote_control_mode==gimbal_CH_width))
		{   
			chassis_flag=0;
			trigger_mode = speed_loop;
		}
		if(Remote_control_mode==dance_CH_width)
		{
			chassis_flag=1;
			trigger_mode = speed_loop;
		}
		if(chassis_flag==0)
		{
			if(Remote_control_mode == chassis_CH_width)
	   {  
				 x_speed=caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
				 y_speed=caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			   r_speed=caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value,r_max_value);  
			   chassis_flag=0;
		  }
	if(Remote_control_mode == gimbal_CH_width)
		 { 
			 	  gimbal_loop=position_loop;
					Kinematics.pitch.target_angle = caculate_gimbal_pitch_angle(i_CH_width,i_initial_value,i_min_value,i_max_value);
					Kinematics.yaw.target_angle = caculate_gimbal_yaw_angle(x_CH_width,x_initial_value,x_min_value,x_max_value);
		 }
		 		switch (trigger_control_mode)                                               //ң�����Ҳ���
				{
					case 1:
					fric_angular=1400;
          static int count_1=1;	
					count_1++;
					if(count_1>100)
					{trigger_speed = 100;
					    count_1=1;           }
					if(motor5.actual_speed<20&&motor5.actual_speed>-20)    						//��ת
					{ 
						static int count_=1;
					  count_++;
						trigger_speed =pow(-1,count_)*50;
						if(count_>100)
							count_=1;
					}
					break;
					case 2:
					trigger_speed = 0;
				  fric_angular=1000;
					/*if(motor5.actual_speed<20&&motor5.actual_speed>-20)    						//��ת
					{ 
						static int count_=1;
					  count_++;
						trigger_speed =pow(-1,count_)*50;
						if(count_>100)
							count_=1;
					}*/
					break;
					case 3:
					trigger_speed = 0;
					fric_angular=1000;
					break;
					
					default:
	      	break;
				 }											

	 }
	else if(chassis_flag==1)
	{
	/***** x,y,z���˶����� *****/
  if(key_board&ws_key)
	{
		if((key_board&W_key) == W_key)
		 x_speed = max_base_linear_speed;
    if((key_board&S_key) == S_key)
		 x_speed = -max_base_linear_speed;
  }
	else
		 x_speed = 0;
	if(key_board&ad_key)
	{
	  if((key_board&A_key) == A_key)
			r_speed = -max_base_rotational_speed;
		if((key_board&D_key) == D_key)
			r_speed = max_base_rotational_speed;	
	}
	else
		 r_speed = 0;
	if(key_board&qe_key)
	{
	  if((key_board&Q_key) == Q_key)
			y_speed = -max_base_linear_speed;
		if((key_board&E_key) == E_key)
			y_speed = max_base_linear_speed;	
	}
	else
		  y_speed = 0;
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
/***** ��� *****/
	if(mouse_pre_left==1)
	{
		trigger_speed=100;
		//bullet_num=1;
		//shoot(bullet_num);
		//bullet_flag=1;
	/*
trigger_mode = speed_loop;
*/		
	}
	else
		trigger_speed=0;
	if(mouse_pre_right==1)
	{
		fric_angular=1295.3;
		//bullet_num=1;
		//shoot(bullet_num);
	}	
	if(mouse_pre_right==0)
		fric_angular=1000;
  if(mouse_x||mouse_y)
	{
	gimbal_loop=speed_loop;
	yaw_angular=-mouse_x*60/350;
	pitch_angular=-mouse_y*42/350;
	}
	else
	{
	yaw_angular=0;
	pitch_angular=0;
	}
	
/***** ��̨��λ��С����ģʽ *****/
	if((key_board&CTRL_key)==CTRL_key)
	{	
	 gimbal_loop=position_loop;
	 Kinematics.yaw.target_angle=0;
	 Kinematics.pitch.target_angle=0;
	}
	if((key_board&F_key)==F_key && aim_flag==0)
	{
		aim_mode = auto_control;
		aim_flag = 1;
	}
	if((key_board&F_key)==Null_key)
		aim_flag=0;
	
	if((key_board&R_key)==R_key && aim_flag_2==0)
	{
	   aim_mode = DJi_Remote_Control;
	   aim_flag_2 = 1;
	}
	if((key_board&R_key)==Null_key)
	   aim_flag_2=0;
	if((key_board&C_key)==C_key && cabin_flag==0 && cabin_flag_1==0)
	{
		steering_engine=1;
    cabin_flag=1;
		cabin_flag_1=1;
	}
	if((key_board&C_key)==C_key && cabin_flag==1 && cabin_flag_1==0)
	{
	  steering_engine=0;
		cabin_flag=0;
		cabin_flag_1=1;
	}
	if((key_board&C_key)==Null_key)
		cabin_flag_1=0;
}
		  
    	r_speed = -r_speed; //ȡ����ʹ��ʱ����תΪ����
			x_speed = -x_speed;
      y_speed = -y_speed;
    dji_remote_assignment();

}

/**
  * @brief  ��ң����ҡ�����ӳ�䵽�����������ٶ���
  * @param  width��ͨ��ֵ 
  *         mid��ͨ���м�ֵ 
  *         min��ͨ�������Сֵ
  *         max��ͨ��������ֵ
  */
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

static float caculate_gimbal_pitch_angle(int width,int mid,int min,int max)
{
	float pwm_can;                         //�˱������ؼ���ó���pwm�������canģʽ�µĻ�е�Ƕ�ֵ
	switch(gimbal_modes)
	{
		case(gimbal_pwm_mode):
			
	   pwm_can=BASIC_PITCH_ANGLE_PWM;
		 if(width>=(mid+2))
		  pwm_can=(BASIC_PITCH_ANGLE_PWM - 1.0*(width-(mid+2))/(max-(mid+2))*210);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_PITCH_ANGLE_PWM + 1.0*((mid-2)-width)/((mid-2)-min)*105);
	   else
		  pwm_can=BASIC_PITCH_ANGLE_PWM;
	
		 break;
		 
		case(gimbal_can_mode):
		 pwm_can=0;
		 if(width>=(mid+2))
		  pwm_can=(0 - 1.0*(width-(mid+2))/(max-(mid+2))*2047);
	   else if(width<=(mid-2))
	    pwm_can=(0 + 1.0*((mid-2)-width)/((mid-2)-min)*1023);
	   else
		  pwm_can=0;		
		 
		 break;
		 default:break;
   }
	return pwm_can;
}

static float caculate_gimbal_yaw_angle(int width,int mid,int min,int max)
{
	float pwm_can;
	switch(gimbal_modes)
	{
		case(gimbal_pwm_mode):
	    pwm_can=BASIC_YAW_ANGLE_PWM;
		 if(width>=(mid+2))
	 	  pwm_can=(BASIC_YAW_ANGLE_PWM - 1.0*(width-(mid+2))/(max-(mid+2))*420);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_YAW_ANGLE_PWM + 1.0*((mid-2)-width)/((mid-2)-min)*420);
	   else
		  pwm_can=BASIC_YAW_ANGLE_PWM;

		 break;
		 
		 case(gimbal_can_mode):
		  pwm_can=0;
		 if(width>=(mid+2))
		  pwm_can=(0 - 1.0*(width-(mid+2))/(max-(mid+2))*4095);
	   else if(width<=(mid-2))
	    pwm_can=(0 + 1.0*((mid-2)-width)/((mid-2)-min)*4095);
	   else
		  pwm_can=0;
		 
		 break;
		 default:break;
		 
   }
		 return pwm_can;
}



