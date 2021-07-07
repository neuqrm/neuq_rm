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
电机数据范围
 ：3508  数据范围 
给定电流 -16384 ~ 0 ~ +16384
转子角度	0~8191 等价	0-360°
转子转速	max=450	RPM 减速比19
实际转矩电流
电机温度	
 ：GM6020数据范围 
给定电压 -30000 ~ 0 ~ +30000
转子角度	0~8191 等价	0-360°
转子转速	max=350	RPM
实际转矩电流
电机温度
 ：P2006 数据范围 
给定电流 -10000 ~ 0 ~ +10000
转子角度	0~8191 等价	0-360°
转子转速	max=450	RPM	减速比36
实际输出转矩
******************************************************/
/**************极限值定义******************************/
//#define MAXANGLE		9000		//控制数据分为两种，当角度值大于此值 认为速度控制
#define MAXSPEED		350.0f			//最大速度值	RPM
#define MAXSPEEDY		40.0f				//6020最大速度RPM yaw
#define MAXSPEEDP		10.0f				//6020最大速度RPM	pitch
#define MAXCURRENT	15000				//最大电流值
#define	MAXVOLTAGE	25000				//最大电压值
#define MAXANGLE		8192				//编码器极限值
#define MIDANGLE		4096				//编码器中值
#define RADSTEP			0.0439			//角度步进->9182
#define YAW_RESETRAD		0.0f		//yaw初始角度
#define YAW_MAXRAD			75.0f			//yaw的最大角度值
#define YAW_MINRAD			(-75.0f)	//yaw的最小角度值
#define PITCH_RESETRAD	0.0f			//pitch初始角度
#define PITCH_MAXRAD		30.0f			//Pitch的最大角度值
#define PITCH_MINRAD		(-30.0f)	//Pitch的最小角度值
//#define YAW_RESETANGLE		2266		//yaw初始角度
//#define YAW_MAXANGLE			4266		//yaw的最大角度值
//#define YAW_MINANGLE			266			//yaw的最小角度值
//#define PITCH_RESETANGLE	6473		//pitch初始角度
//#define PITCH_MAXANGLE		8190		//Pitch的最大角度值
//#define PITCH_MINANGLE		4756		//Pitch的最小角度值
#define YAW_RESETANGLE		5391		//yaw初始角度
#define YAW_MAXANGLE			6100		//yaw的最大角度值
#define YAW_MINANGLE			4600		//yaw的最小角度值
#define PITCH_RESETANGLE	7500		//pitch初始角度
#define PITCH_MAXANGLE		6831		//Pitch的最大角度值
#define PITCH_MINANGLE		6813		//Pitch的最小角度值
#define RAD2ANGLE					22.755	//8192->360的转化
#define SHOCK_STEP				10			//稳定临界震荡值 可以实验测量

#define TRIGGER_SHOOTSPEED		ShootLimitData.LimitTriggerRpm	 //trigger点射转子转速
#define TRIGGER_ANGLESTEP			2048 //trigger角度控制步进值 转子的1/4圈
#define TRIGGER_RADSTEP				32.727f	//点射步进值
#define TRIGGER_TOOTH					12   //realangle * M2006VAL / 拨弹轮齿数 * ANGLESTEP 11齿

#define FRICRPM_RESET		400	//银燕4114和snail的初始占空比不一样
#define FRICRPM_LOW			ShootLimitData.LimitFricPluse
#define FRICRPM_HIGH		2000	//摩擦轮转速 步兵
//#define FRICRPM_RESET		1000
//#define FRICRPM_LOW			1500
//#define FRICRPM_HIGH		2000	//摩擦轮转速 英雄


/********************战车相关参数**************************************/
#define M2006VAL			36.0f		//M2006速度系数 与减速比有关	36
#define M3508AVAL			19.2f		//M3508速度系数 与减速比有关	18
#define GM6020VAL			1.0f		//GM6020速度系数 无减速比
#define CHASSIS_LONG	365			
#define CHASSIS_WIDE	500			//455+45 麦轮厚45mm
#define MECANUM_DIAMETER	300	//麦轮直径 150 

#endif /*__ROBOTDATA_H*/
