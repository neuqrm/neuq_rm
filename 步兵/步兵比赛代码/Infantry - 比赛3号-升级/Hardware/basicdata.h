#ifndef __ROBOTDATA_H
#define	__ROBOTDATA_H

#include "math.h"
#include "string.h"
#include <stdio.h>

typedef struct 
{
	float Xrpm;
	float Yrpm;
	float Zrpm;
}Chassis_Rpm_stc;
typedef struct 
{
	float Yaw_rpm;
	float Yaw_rad;
}Gimbal_Yaw_stc;
typedef struct 
{
	float Pitch_rpm;
	float Pitch_rad;
}Gimbal_Pitch_stc;
typedef struct 
{
	float Trigger_rpm;
	float Fric_rpm;
	float Trigger_rad;
}Gimbal_Trigger_stc;

/******************************************************
������ݷ�Χ
 ��3508  ���ݷ�Χ 
�������� -16384 ~ 0 ~ +16384
ת�ӽǶ�	0~8191 �ȼ�	0-360��
ת��ת��	max=450	RPM ���ٱ�19
ʵ��ת�ص���
����¶�	
 ��GM6020���ݷ�Χ 
������ѹ -30000 ~ 0 ~ +30000
ת�ӽǶ�	0~8191 �ȼ�	0-360��
ת��ת��	max=350	RPM
ʵ��ת�ص���
����¶�
 ��P2006 ���ݷ�Χ 
�������� -10000 ~ 0 ~ +10000
ת�ӽǶ�	0~8191 �ȼ�	0-360��
ת��ת��	max=450	RPM	���ٱ�36
ʵ�����ת��
******************************************************/
/**************����ֵ����******************************/
//#define MAXANGLE		9000		//�������ݷ�Ϊ���֣����Ƕ�ֵ���ڴ�ֵ ��Ϊ�ٶȿ���
#define MAXSPEED		350.0f			//����ٶ�ֵ	RPM
#define MAXSPEEDY		40.0f				//6020����ٶ�RPM yaw
#define MAXSPEEDP		10.0f				//6020����ٶ�RPM	pitch
#define MAXCURRENT	15000				//������ֵ
#define	MAXVOLTAGE	25000				//����ѹֵ
#define MAXANGLE		8192				//����������ֵ
#define MIDANGLE		4096				//��������ֵ
#define RADSTEP			0.0439			//�ǶȲ���->9182
#define YAW_RESETRAD		0.0f		//yaw��ʼ�Ƕ�
#define YAW_MAXRAD			75.0f			//yaw�����Ƕ�ֵ
#define YAW_MINRAD			(-75.0f)	//yaw����С�Ƕ�ֵ
#define PITCH_RESETRAD	0.0f			//pitch��ʼ�Ƕ�
#define PITCH_MAXRAD		30.0f			//Pitch�����Ƕ�ֵ
#define PITCH_MINRAD		(-30.0f)	//Pitch����С�Ƕ�ֵ
//#define YAW_RESETANGLE		2266		//yaw��ʼ�Ƕ�
//#define YAW_MAXANGLE			4266		//yaw�����Ƕ�ֵ
//#define YAW_MINANGLE			266			//yaw����С�Ƕ�ֵ
//#define PITCH_RESETANGLE	6473		//pitch��ʼ�Ƕ�
//#define PITCH_MAXANGLE		8190		//Pitch�����Ƕ�ֵ
//#define PITCH_MINANGLE		4756		//Pitch����С�Ƕ�ֵ
#define YAW_RESETANGLE		5391		//yaw��ʼ�Ƕ�
#define YAW_MAXANGLE			6100		//yaw�����Ƕ�ֵ
#define YAW_MINANGLE			4600		//yaw����С�Ƕ�ֵ
#define PITCH_RESETANGLE	7500		//pitch��ʼ�Ƕ�
#define PITCH_MAXANGLE		6831		//Pitch�����Ƕ�ֵ
#define PITCH_MINANGLE		6813		//Pitch����С�Ƕ�ֵ
#define RAD2ANGLE					22.755	//8192->360��ת��
#define SHOCK_STEP				10			//�ȶ��ٽ���ֵ ����ʵ�����

#define TRIGGER_SHOOTSPEED		ShootLimitData.LimitTriggerRpm	 //trigger����ת��ת��
#define TRIGGER_ANGLESTEP			2048 //trigger�Ƕȿ��Ʋ���ֵ ת�ӵ�1/4Ȧ
#define TRIGGER_RADSTEP				32.727f	//���䲽��ֵ
#define TRIGGER_TOOTH					12   //realangle * M2006VAL / �����ֳ��� * ANGLESTEP 11��

#define FRICRPM_RESET		400	//����4114��snail�ĳ�ʼռ�ձȲ�һ��
#define FRICRPM_LOW			ShootLimitData.LimitFricPluse
#define FRICRPM_HIGH		2000	//Ħ����ת�� ����
//#define FRICRPM_RESET		1000
//#define FRICRPM_LOW			1500
//#define FRICRPM_HIGH		2000	//Ħ����ת�� Ӣ��


/********************ս����ز���**************************************/
#define M2006VAL			36.0f		//M2006�ٶ�ϵ�� ����ٱ��й�	36
#define M3508AVAL			19.2f		//M3508�ٶ�ϵ�� ����ٱ��й�	18
#define GM6020VAL			1.0f		//GM6020�ٶ�ϵ�� �޼��ٱ�
#define CHASSIS_LONG	365			
#define CHASSIS_WIDE	500			//455+45 ���ֺ�45mm
#define MECANUM_DIAMETER	300	//����ֱ�� 150 

#endif /*__ROBOTDATA_H*/
