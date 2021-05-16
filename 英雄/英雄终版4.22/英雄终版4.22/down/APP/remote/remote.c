#include "remote.h"
#include "motor.h"
#include "bsp_led.h"
#include "remote_code.h"
#include "bsp_dbus.h"
#include "kinematic.h"
#include "fric.h"
#include "stm32f4xx_tim.h"
#include "gimbal.h"
#include "imuReader.h"              
#include <math.h>
#include "mode.h"
#include "delay.h"
#include "My_Init.h"

//����remote_code.c����м�����ͼ��㺯��
extern float x_speed,y_speed,r_speed,trigger_speed,theta,trigger_angle,yaw_angle1,pitch_angle1;
extern float cx_speed,cy_speed,fric_angular,yaw_angular,pitch_angular;
extern int flag;
extern int trigger_flag;
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);
void set_trigger_control(void);

int CONTROL_MODE=0; //�������Կ��� 0Ϊң�������� 1Ϊ���Կ���

void control_remote_mode_choose(void)
{
	x_speed=0;
	y_speed=0;
	r_speed=0;
	if(CONTROL_MODE==0)
	DJI_Remote_Control();
	else PC_Remote_Control();
	
}
/**************************************************
ս������DJIң��������
**************************************************/
void DJI_Remote_Control(void)
{
		DJI_Remote_Chassis();
		DJI_Remote_Gimball();
		DJI_Remote_Action();
	    if((control_mode) == DJi_Remote_Control)
		{
			y_speed = y_speed;
			r_speed = -r_speed; //ȡ����ʹ��ʱ����תΪ����
		}
		dji_remote_assignment();  //������ң�ؼ������ݸ�ֵ��ִ�в�����robomove��
        set_trigger_control();
}

/**************************************************
ս������PCң��������
**************************************************/
void PC_Remote_Control(void)
{
		PC_Remote_Chassis();
		PC_Remote_Gimball();
		PC_Remote_Action();
	    fric_angular=1000;
	    dji_remote_assignment();
	   
	    Kinematics.trigger.target_angle+=trigger_angle; 
}
/***************************************************
ң�������̿��ƴ���`
���ܣ���ҡ�˻ش�����ֵת��Ϊ���̵��˶�
���ã����̿��ƴ��롢ң������������
��ʽ���ٶȿ���
������� ����Ϊ�����˶�
����ƽ���˶�
������ת�˶�
***************************************************/

void DJI_Remote_Chassis(void)
{
   x_speed=caculate_linear_speed(DJI_Motion_X,y_initial_value,y_min_value,y_max_value);
   y_speed=caculate_linear_speed(DJI_Motion_Y,x_initial_value,x_min_value,x_max_value);
   r_speed=caculate_rotational_speed(DJI_Motion_Z,z_initial_value,z_min_value,z_max_value);  
}
/**************************************************
ң������̨���ƴ���
���ܣ���ҡ�˺Ͳ��ֻش�����ֵת��Ϊ��̨���˶�
���ã���̨���ƴ��롢ң������������
��ʽ���Ƕȿ���
������� ����Ϊ��̨�˶�
����pitch�˶�
����yaw�˶�
**************************************************/
//ģʽ���������������ģʽ������ѡ��
void DJI_Remote_Gimball(void)
{
	switch(gimbal_modes)
	{
				case(gimbal_pwm_mode):  				//pwmģʽ�¿�����̨ת��

			  	pwm_pulse_p=caculate_gimbal_pitch_angle(DJI_Motion_Pitch,i_initial_value,i_min_value,i_max_value);
				  pwm_pulse_y=caculate_gimbal_yaw_angle(DJI_Motion_Yaw,x_initial_value,x_min_value,x_max_value);
				
				break;
				case(gimbal_can_mode):           //canģʽ�¿�����̨ת��
					Kinematics.pitch.target_angle = caculate_gimbal_pitch_angle(DJI_Motion_Pitch,i_initial_value,i_min_value,i_max_value);
					Kinematics.yaw.target_angle = caculate_gimbal_yaw_angle(DJI_Motion_Yaw,x_initial_value,x_min_value,x_max_value);
                break;
				default:break;
     }
}
/**************************************************
ս����������
���ܣ������Ƹ����ݣ��ж�ս������
���ã���̨�����̿��ƴ��룬ң������������
��ʽ���߼�����
������� ����Ϊս������
	Action_Rest			((3<<8)|3)	//��λ״̬ �޶���
	Action_Shoot_H	((1<<8)|2)	//���ٿ���	Ħ�������ٽϴ󣬲����ָ���ת��
	Action_Fire_H		((1<<8)|1)	//���ٵ���	Ħ�������ٽϴ󣬲����ֽǶ�ת��
	Action_Shoot_L	((2<<8)|2)	//���ٿ���	Ħ�������ٽ�С�������ָ���ת��
	Action_Fire_L		((2<<8)|1)	//���ٵ���	Ħ�������ٽ�С�������ֽǶ�ת��
	Action_Unknown1	((1<<8)|3)	//������������С���ݵ�
	Action_Unknown2	((2<<8)|3)	//
	Action_Unknown3	((3<<8)|1)	//
	Action_Unknown4	((3<<8)|2)	//
**************************************************/
void DJI_Remote_Action(void)
{
	switch(DJI_Action_Flag)
	{
		case DJI_Action_Reset:;//��λ״̬ Ħ���ֲ����ֶ��� ��
		{
			LED_Control(0x11);
			LASER_OFF;
			
			trigger_angle = 0;
			fric_angular=1000;
		}
		break;
		case DJI_Action_Shoot_H:
		{
			LED_Control(0x22);
			
			trigger_angle = 0;
			fric_angular=1300;
			
		}
		break;
		case DJI_Action_Fire_H:
		{
			LED_Control(0x44);
			fric_angular=1300;
					
			if(trigger_angle!=1100&&trigger_angle!=1800) trigger_flag=0;  
			trigger_angle = 1100; 
		}
		break;
		case DJI_Action_Shoot_L:
		{
			LED_Control(0x88);
			fric_angular=1300;
					
			if(trigger_angle!=900&&trigger_angle!=1800) trigger_flag=0;  
			trigger_angle = 1800; 
		}
		break;
		case DJI_Action_Fire_L:
		{
			LED_Control(0x81);
			trigger_angle = 0;
			fric_angular=1300;
		}
		break;

		case DJI_Action_Unknown1:
		{
			LED_Control(0x24);
			LASER_ON;
			trigger_angle = 0;
			fric_angular=0;
		}
		break;
		case DJI_Action_Unknown3://�������Կ���ģʽ  3 1
		{
			LED_Control(0x18);
			CONTROL_MODE=1;
		}
		break;
		case DJI_Action_Unknown4:
		{
			LED_Control(0x33);
		}
		break;
		default:	break;					
		
	}
	
}

int16_t PC_temp_rpm_up(int16_t PC_temp_rpm,uint8_t XYZFlag)
{
	int16_t temp_rpm_up = PC_temp_rpm;
/*******************�����ٶȲ�ͬ����***********************/
//	if (temp_rpm_up > 0)
//	{
//		temp_rpm_up = temp_rpm_up + RPM_STEP;
//		if (temp_rpm_up > temp_rpm_max)
//			temp_rpm_up = temp_rpm_max;
//	}
//	else
//	{
//		temp_rpm_up = temp_rpm_up - RPM_STEP;
//		if (temp_rpm_up < temp_rpm_min)
//			temp_rpm_up = temp_rpm_min;
//	} 
/******************�����ٶ���ͬ����***********************/
	if (XYZFlag)
	{
		temp_rpm_up = temp_rpm_up + RPM_STEP;
		if (temp_rpm_up > XYRPM_MAX)
			temp_rpm_up = XYRPM_MAX;
	}
	else
	{
		temp_rpm_up = temp_rpm_up + RPM_STEP;
		if (temp_rpm_up > ZRPM_MAX)
			temp_rpm_up = ZRPM_MAX;
	}
	return (temp_rpm_up);
}
int16_t PC_temp_rpm_down(int16_t PC_temp_rpm,uint8_t XYZFlag)
{
	int16_t temp_rpm_down = PC_temp_rpm;
	if (XYZFlag)
	{
		temp_rpm_down = temp_rpm_down - RPM_STEP;
		if (temp_rpm_down < 0)
			temp_rpm_down = 0;
	}
	else
	{
		temp_rpm_down = temp_rpm_down - RPM_STEP;
		if (temp_rpm_down < 0)
			temp_rpm_down = 0;
	}
	return (temp_rpm_down);
}
/*********************************************************************
PC���̿��ƴ���
���ܣ������̻ش�����ֵת��Ϊ���̵��˶�
���ã����̿��ƴ��롢PC��������
��ʽ���ٶȿ���
������� ����Ϊ�����˶�
		W
A		S		D
ʵ��ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
Shift Ϊ���ٰ�ť �����ά�˶�
Shift +			 	W
					A		S		D
ʵ�ֶ�Ӧ����ļ���ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
Ctrl Ϊ���ٰ�ť �����ά�˶�
Ctrl +			 	W
					A		S		D
ʵ�ֶ�Ӧ����ļ���ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
E��						�ٶ�ֵ��ʼ��
Shift	+	E��		ɲ����ͣ
Shift + Ctrl	��̨�Ƕȸ�λ
*********************************************************************/
void PC_Remote_Chassis(void)
{
	int16_t PC_temp_rpm_X=0,PC_temp_rpm_Y=0,PC_temp_rpm_Z=0;//��������м��� ��Ϊ��ֵ
	int16_t PC_set_rpm_X=0,PC_set_rpm_Y=0,PC_set_rpm_Z=0;		//����ṹ���������
	switch (PC_KEY_Flag & 0x0004)
	{
		case PC_Motion_XP://ǰ��
		{
			PC_temp_rpm_X=XRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_X = PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag);
			else
				PC_set_rpm_X = PC_temp_rpm_X;
		}break;
		case PC_Motion_XN://����
		{
			PC_temp_rpm_X=XRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_X = (-PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = (-PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag));
			else
				PC_set_rpm_X = (-PC_temp_rpm_X);
		}break;
		case PC_Motion_YP://����ƽ��
		{
			PC_temp_rpm_Y=YRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Y = PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag);
			else
				PC_set_rpm_Y = PC_temp_rpm_Y;
		}break;
		case PC_Motion_YN://����ƽ��
		{
			PC_temp_rpm_Y=YRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Y = (-PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = (-PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag));
			else
				PC_set_rpm_Y = (-PC_temp_rpm_Y);
		}break;
		case PC_Motion_ZP://����ת
		{
			PC_temp_rpm_Z=ZRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Z = PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag);
			else
				PC_set_rpm_Z = PC_temp_rpm_Z;
		}break;
		case PC_Motion_ZN://����ת
		{
			PC_temp_rpm_Z=ZRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Z = (-PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = (-PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag));
			else
				PC_set_rpm_Z = (-PC_temp_rpm_Z);
		}break;
		case PC_Motion_Reset:
		{
			PC_temp_rpm_X = 0;
			PC_temp_rpm_Y = 0;
			PC_temp_rpm_Z = 0;
		}break;//�����ٶȼ���������
		case PC_Action_Reset:
		{
			motor_Init_angle(); 
		}break;//��̨�ǶȻ���
		default:
		{
			/*
			if(flag==0)
			{
				fric_angular=1200;
				flag=1;
			}
			else 
			{
				fric_angular=1000;
				flag=0;
			}
			*/
		}break;//����������
	}
	/*****************���ٶ�ֵ����������API***********************/
	x_speed=PC_set_rpm_X;
	y_speed=PC_set_rpm_Y;
	r_speed=PC_set_rpm_Z;
}
/******************************************************************
�����̨����
���ܣ������ƫ�Ƶ��ٶ�ת��Ϊ��̨yaw��pitch��ת���ٶ�
���ã���̨���ƴ��롢PC��������
��ʽ���ٶȿ���
������� ����Ϊ��̨�˶�
mouse X ��̨��yaw����ת��		//����λ 360��
mouse Y ��̨��pitch����ת��	//����λ ������
mouse Z ����
mouse left_key 	�������
mouse right_key	�Ҽ�����
*******************************************************************/
void PC_Remote_Gimball(void)
{
	if(PC_Mouse_Left) //����
	{
		trigger_angle = 900;
		fric_angular=1600;   //1300
	}
	if(PC_Mouse_Right) //����
	{
		trigger_angle = 2700;
		fric_angular=1600;   //1300
	}
	if(PC_Mouse_X||PC_Mouse_Y)
	{
		gimbal_loop=speed_loop;
		yaw_angular=PC_Mouse_X*40/32767;
		pitch_angular=PC_Mouse_Y*20/32767;
	}	
	else gimbal_loop=position_loop;
}

/******************************************************************
�����̨����
���ܣ�����갴����ֵת��Ϊ��̨����ָ��
���ã���̨���ƴ��롢PC��������
��ʽ���߼�����
������� ����Ϊ��̨����
mouse left_key 	�������
mouse right_key	�Ҽ�����
*******************************************************************/
void PC_Remote_Action(void)
{
	if (PC_Mouse_Left & ~PC_Mouse_Right)
	{
		
	}//��������
	else if (PC_Mouse_Right & PC_Mouse_Left)
	{
		
	}//����
	else
	{
		
	}//�޶���
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




