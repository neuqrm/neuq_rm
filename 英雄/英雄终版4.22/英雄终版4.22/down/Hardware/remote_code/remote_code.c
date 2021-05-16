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
#include "shoot.h"
#include "bsp_dbus.h"
#include "referee.h"
#include "algorithm.h"
//�ڲ�ȫ�ֱ������������
float x_speed=0,y_speed=0,r_speed=0,trigger_speed=0,theta=0,alpha=0,trigger_angle=0,yaw_romote_angle=0,pitch_romote_angle=0;
float x_speed_w=0,y_speed_w=0;//Ťƨ��ʱ�������ٶ�
int dance_switch=0;
float cx_speed=0,cy_speed=0,fric_angular=0,yaw_angular=0,pitch_angular=0,trigger_angle1=0;
int flag = 0;
extern IMU_DATA imu_data;
//�ڲ���������
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);
float caculate_yaw_angle(int width,int mid,int min,int max);
int abs_romote(int a)
{
	if(a<0) return -a;
	else return a;
}
int trigger_flag=1;//�˱��Ϊ��ʵ���ӵ��������������Ⲩ����ת���Ƕ��ۼ�
/*
    �˺�������ʵ�ֲ����ֵ����ӵ��Ƕ�����
*/

void set_trigger_control(void)
{

		if(trigger_flag==0) 
		{
			Kinematics.trigger.target_angle+=trigger_angle; 
			trigger_flag=1;
		}
}	
/**
    �жϵ�ǰ�ȼ�
    �Դ����ı�Ħ���ֲ����ٶ�
    �����ȼ�֮��ǹ����������֮��Ϊ100
*/
int last_limit=0,actual_level=0;
void level_return(void)
{
	int shoot_limit=Get_Shoot_Heatlimit();  //�õ���ǰ����
	if(shoot_limit-last_limit>=40) actual_level++;  //��ֹ���ݶ���
	last_limit=shoot_limit;
}

int mode_control_flag=1;
//������ǵ�ǰ����ģʽ    1 Ϊң���� 0Ϊ ���̿���
int stuck_flag=1;//  1û�� 0 ��
//�������
/**
  * @brief ������ң������ϴ���

  *  ��߲��˲����м�����ң��������ģʽ
  *  ������Ϊ���̿���ģʽ
  */
void remote_control_test()
{
	if(Remote_control_mode == chassis_CH_width)
	{
    dance_switch=0;
		alpha=0;    	
    theta=Get_gimbal_theta(); 		
		x_speed=caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
		y_speed=caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
		r_speed=caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value,r_max_value);  
		
		//Kinematics.pitch.target_angle = caculate_gimbal_pitch_angle(i_CH_width,i_initial_value,i_min_value,i_max_value);
		//Kinematics.yaw.target_angle = caculate_gimbal_yaw_angle(DJI_Motion_Yaw,x_initial_value,x_min_value,x_max_value);

		
		switch (trigger_control_mode)                                               //ң�����Ҳ���
		{
			case 1:
		  fric_angular=100;					
		  if(trigger_angle!=1000&&mode_control_flag==1) trigger_flag=0;  
			if(mode_control_flag==0)  trigger_flag=0;
			trigger_angle = 1000; 
			break;
			case 2:
						trigger_angle = 0;
				    fric_angular=0;
					break;
					case 3:
						
					trigger_angle = 0;
					fric_angular=100;
					break;
			
					default:
	      	    break;
		}											

		r_speed = -r_speed; //ȡ����ʹ��ʱ����תΪ����
		
		mode_control_flag=1;
	}
	
	/***** x,y,z���˶����� *****/
	else if(Remote_control_mode==1)   //�󲦸˸�λ
	{ 
		dance_switch=0;
		alpha=0;    	
    theta=Get_gimbal_theta(); 
		yaw_romote_angle=0;
	  pitch_romote_angle=0;
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
			r_speed = max_base_rotational_speed;
		if((key_board&D_key) == D_key)
			r_speed = -max_base_rotational_speed;	
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
		level_return();
		if(actual_level<2)
		{
			fric_angular=100;
			if(trigger_angle!=1000&&mode_control_flag==0) trigger_flag=0;  
			if(mode_control_flag==1) trigger_flag=0;  
			trigger_angle = 1000; 
		}
		 else if(actual_level>=2)
	 {
			 fric_angular=100;
			if(trigger_angle!=1100&&mode_control_flag==0) trigger_flag=0;  
			if(mode_control_flag==1) trigger_flag=0;  
			trigger_angle = 1000; 
			if(abs(motor5.vpid.PID_OUT)>1000) stuck_flag=0;//���ڿ���״̬
		 }				 
	}
   if(mouse_pre_left==0)
		trigger_angle=0;
	if(mouse_pre_right==1)
	{
		//trigger_angle=0;
		fric_angular=100;
	}	
	else if (mouse_pre_left==0&&mouse_pre_right==0)
	fric_angular=0;
  if(mouse_x||mouse_y)
	{
	yaw_romote_angle=-mouse_x*140/32767;
	//�Ƕ���λ
	if(yaw_romote_angle>=1.047) yaw_romote_angle=1.047;  //PI/3
	if(yaw_romote_angle<=-1.047)  yaw_romote_angle=-1.047;
		
	//�Ƕ���λ
	pitch_romote_angle=mouse_y*4000/32767;
	if(pitch_romote_angle>=35) pitch_romote_angle=35;
	if(pitch_romote_angle<=-35)  pitch_romote_angle=-35;
	}
	else 
	{
		//gimbal_loop=position_loop;
		yaw_romote_angle=0;
	  pitch_romote_angle=0;
	}
	
/***** ��̨��λ��С����ģʽ *****/
	if((key_board&CTRL_key)==CTRL_key)
	{	
	 gimbal_loop=position_loop;
   Kinematics.yaw.target_angle=0;     
	 Kinematics.pitch.target_angle=0; 
	}
/****          ������           ****/
	if((key_board & F_key)==F_key)
	{
		if(stuck_flag==1)
		Kinematics.trigger.target_angle-=100; 
		stuck_flag=0;
	}
	else 
	{
		stuck_flag=1;
		//actual_level=1;
	}
	mode_control_flag=0;
	Kinematics.yaw.target_angle+=yaw_romote_angle;  
	Kinematics.pitch.target_angle+=pitch_romote_angle;   
if(Kinematics.yaw.target_angle>1.382)	Kinematics.yaw.target_angle=1.382f;
if(Kinematics.yaw.target_angle<-1.382)	Kinematics.yaw.target_angle=4.887f;	
if(Kinematics.yaw.target_angle<0)  Kinematics.yaw.target_angle=2*PI+Kinematics.yaw.target_angle;
	  
	}
	else if(Remote_control_mode==2)//��˵�λ   Ťƨ��/С����
	{      if(dance_switch==0) { imu_data.start_dance_yaw=imu_data.yaw;dance_switch=1;r_speed=1.2f;}
		     Kinematics.yaw.target_angle=caculate_yaw_angle(r_CH_width,x_initial_value,x_min_value,x_max_value);
	       //r_speed=0.7f;
		     theta=Get_gimbal_theta();   
		     alpha=Get_chassis_alpha();          //���̶Ե�ת���������� 
		     if(theta>0.95&&theta<PI)  r_speed=1.2f;	
	        if(theta>PI&&theta<5.33) r_speed=-1.2f;
         //r_speed=caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value,r_max_value);  	
	       x_speed_w=caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);  //�����̨�������ٶ�
				 y_speed_w=caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);		        
		     x_speed = x_speed_w*cos(theta) + y_speed_w*sin(theta); 
		     y_speed = y_speed_w*cos(theta) - x_speed_w*sin(theta);
	
	}		
    set_trigger_control();
	  dji_remote_assignment();
}

/**********************************************/
float Get_chassis_alpha()  //С����ʱ���̶Ե�ת�� 0-2PI
{
float temp,angle;

	if(imu_data.yaw<imu_data.start_dance_yaw)  temp=imu_data.yaw+(2*PI);
	else temp=imu_data.yaw;
	angle=temp-imu_data.start_dance_yaw;
	return angle;
	
}
float Get_gimbal_theta() //С������̨�Ե���ת��  0-2PI
{
float temp,angle;
	if(gimbal_y.actual_angle<BASIC_YAW_ANGLE_CAN)  temp=gimbal_y.actual_angle+8191;
	else temp=gimbal_y.actual_angle;
	
	if(temp<=BASIC_YAW_ANGLE_CAN+4096)
	angle=(temp-BASIC_YAW_ANGLE_CAN)/8191*2*PI/2;
	else if(temp>BASIC_YAW_ANGLE_CAN+4096)
	angle=2*PI-(PI-((temp-(BASIC_YAW_ANGLE_CAN+4096))/8191*2*PI))/2	;
	return angle;
}
	

/**
  * @brief  ��ң����ҡ�����ӳ�䵽�����������ٶ���
  * @param  width��ͨ��ֵ 
  *         mid��ͨ���м�ֵ 
  *         min��ͨ�������Сֵ
  *         max��ͨ��������ֵ
  */

/**����С��****/
static float caculate_yaw_angle(int width,int mid,int min,int max)
{
float angle;
if(width>=(mid+2))		
    angle=(1.0*(width-(mid+2))/(max-(mid+2))*PI*0.5);	
else if(width<=(mid-2))
	   angle=2*PI-(1.0*(width-(mid-2))/(-(mid-2)+min)*PI*0.5);
 
  return angle;

}


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
  return speed*3.5;                
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
		  pwm_can=(BASIC_PITCH_ANGLE_CAN - 1.0*(width-(mid+2))/(max-(mid+2))*2447);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_PITCH_ANGLE_CAN + 1.0*((mid-2)-width)/((mid-2)-min)*1423);
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



