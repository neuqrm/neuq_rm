#ifndef _REMOTE_CODE_H_
#define _REMOTE_CODE_H_
#include "sys.h"

//Control_Mode��1λ   FSΪ0   DJiΪ1
#define FS_Remote_Control			0x0
#define DJi_Remote_Control		0x1

//Control_Mode��2λ  autoΪ1   remoteΪ0
#define auto_control					0x2
#define remote_control        0x1
#define ROS_control						0x4

//Control_Mode��1λң����ѡ�񣬵ڶ�λģʽѡ��
#define control_mode     (auto_control | DJi_Remote_Control)			//�Զ�+��ң����

extern u8 Control_Mode;

/*
  //��ң����������Χ
  ch0:364-1024-1684
	ch1:364-1024-1684
	ch2:364-1024-1684
  ch3:364-1024-1684
  s1: 1-3-2
  s2: 1-3-2
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
	#define trigger_CH_width      rc.s2          //�����ֿ���
	
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
	#define remote_min_value     0.5		//remote��ΧӦ����stop


void Remote_Control(void);
float x_max_speed_caculator(float x);
float y_max_speed_caculator(float y);
float z_max_speed_caculator(float z);


#endif
