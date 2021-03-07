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
#include "fric.h"
#include "stm32f4xx_tim.h"
#include "gimbal.h"
#include "imuReader.h"              
#include <math.h>
#include "mode.h"
#include "delay.h"

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
	if(Remote_control_mode>remote_min_value && Remote_control_mode<remote_max_value)		//�ж�ң�����Ƿ���
		control_mode = DJi_Remote_Control;								//��־λ��Ϊң��ģʽ
	else
		control_mode = auto_control;							//��־λ��Ϊ�Զ�ģʽ
						
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
				  fric1_on(1500);
				  fric2_on(1500);
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
				  fric1_on(1000);
				  fric2_on(1000);
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
				  fric1_on(1000);
				  fric2_on(1000);
					break;
					
					default:
	      	break;
				 }											

		if((control_mode) == DJi_Remote_Control)
		{
			y_speed = y_speed;
			r_speed = -r_speed; //ȡ����ʹ��ʱ����תΪ����
		}
			dji_remote_assignment();  //������ң�ؼ������ݸ�ֵ��ִ�в�����robomove��
  
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
		 pwm_can=BASIC_PITCH_ANGLE_CAN;
		 if(width>=(mid+2))
		  pwm_can=(BASIC_PITCH_ANGLE_CAN - 1.0*(width-(mid+2))/(max-(mid+2))*2047);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_PITCH_ANGLE_CAN + 1.0*((mid-2)-width)/((mid-2)-min)*1023);
	   else
		  pwm_can=BASIC_PITCH_ANGLE_CAN;		
		 
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
		  pwm_can=BASIC_YAW_ANGLE_CAN;
		 if(width>=(mid+2))
		  pwm_can=(BASIC_YAW_ANGLE_CAN - 1.0*(width-(mid+2))/(max-(mid+2))*4095);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_YAW_ANGLE_CAN + 1.0*((mid-2)-width)/((mid-2)-min)*4095);
	   else
		  pwm_can=BASIC_YAW_ANGLE_CAN;
		 
		 break;
		 default:break;
		 
   }
		 return pwm_can;
}



