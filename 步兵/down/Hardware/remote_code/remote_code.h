#ifndef _REMOTE_CODE_H_
#define _REMOTE_CODE_H_
#include "sys.h"

/*
  //��ң����������Χ
  ch0:364-1024-1684
	ch1:364-1024-1684
	ch2:364-1024-1684
  ch3:364-1024-1684
  s1: 1-3-2
  s2: 1-3-2
	sw  
*/
	
//��ң����

  #define x_CH_width            rc.ch0         //x����ͨ������   ��ҡ������
	#define y_CH_width            rc.ch1         //y����ͨ������   ��ҡ������
	#define r_CH_width            rc.ch2         //r����ͨ������   ��ҡ������
	#define i_CH_width            rc.ch3         //��̨����ͨ������ ��ҡ������
	#define Remote_control_mode   rc.s1
	#define dance_CH_width        2   					 //��������				s1��������
	#define chassis_CH_width      3  				     //ң�ص���    s1�����м�   1 3  2      ������̨  �м����  ��������
	#define gimbal_CH_width       1              //ң����̨    s1����  1
	#define trigger_control_mode      rc.s2          //�����ֿ���
	
	#define mouse_x               RC_Ctl.mouse.x     //ң��������ͨ�����ƣ�������
	#define mouse_y               RC_Ctl.mouse.y
	#define mouse_z               RC_Ctl.mouse.z
  #define mouse_pre_left        RC_Ctl.mouse.press_l
	#define mouse_pre_right       RC_Ctl.mouse.press_r
	#define key_board             RC_Ctl.key.v
	/*���¶����е���ֵ��ͨ��watch�й۲��ͨ��ֵ����*/
	
  //�������ʼֵ
	#define x_initial_value       1024            
	#define y_initial_value       1024
	#define r_initial_value       1024
	#define i_initial_value       1024
	//�����������Сֵ
	#define x_max_value           1684             
	#define x_min_value           364
	#define y_max_value           1684
	#define y_min_value           364
	#define r_max_value           1684
	#define r_min_value           364
	#define i_max_value           1684
	#define i_min_value           364
	//��ֵ������
	#define stop_max_value				2.5
	#define stop_min_value				1.5
	#define remote_max_value      4
	#define remote_min_value      0.5		//remote��ΧӦ����stop
	
	#define W_key                0x000001
	#define S_key                0x000002
	#define ws_key               0x000003
	
	#define A_key                0x000004
	#define D_key                0x000008
	#define ad_key               0x00000C
	
	#define Q_key                0x000040
	#define E_key                0x000080
	#define qe_key               0x0000C0
	
	#define SHIFT_key            0x000010
	#define CTRL_key             0x000020
	
	#define F_key                0x000200
	#define Null_key             0x000000
	
	#define R_key                0x100000
  
	#define C_key                0x200000
	#define dji_remote_assignment() \
	do{ \
		Kinematics.target_velocities.linear_x=x_speed; \
		Kinematics.target_velocities.linear_y=y_speed; \
		Kinematics.target_velocities.angular_z=r_speed; \
		Kinematics.fric.target_angular=fric_angular;     \
		Kinematics.yaw.target_angular = yaw_angular; \
		Kinematics.pitch.target_angular = pitch_angular; \
		Kinematics.trigger.target_angular = trigger_speed; \
	}while(0)                                        \

	#define fric_shoot_assignment() \
do{ \
	Kinematics.fric.target_angular=fric_angular;     \
	Kinematics.trigger.target_angle=trigger_angle; \
}while(0)  

	#define remote_ch_init() \
do{ \
		rc.ch0=1024;\
		rc.ch1=1024;\
		rc.ch2=1024;\
		rc.ch3=1024;\
}while(0)\

void Remote_Control(void);
void remote_control_test(void);
float x_max_speed_caculator(float x);
float y_max_speed_caculator(float y);
float z_max_speed_caculator(float z);

extern float trigger_angle;
extern float trigger_speed;

#endif
