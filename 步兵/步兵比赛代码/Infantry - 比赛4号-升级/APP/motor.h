#ifndef __MOTOR_H
#define	__MOTOR_H

#include "bsp_basic_tim.h"
#include "bsp_advance_tim.h"
#include "CAN_receive.h"
#include "debug.h"
#include "motion.h"
#include "basicdata.h"
#include "imu.h"
#include "pid.h"
#include "filter.h"

typedef struct 
{
	int16 MoterID1_rpm;
	int16 MoterID2_rpm;
	int16 MoterID3_rpm;
	int16 MoterID4_rpm;
}Motor_Chassis_Rpm_stc;
typedef struct 
{
	int16		Yaw_rpm;
	uint16	Yaw_angle;
}Motor_Gimbal_Yaw_stc;
typedef struct 
{
	int16 	Pitch_rpm;
	uint16	Pitch_angle;
}Motor_Gimbal_Pitch_stc;
typedef struct 
{
	int16		Trigger_rpm;
	uint16	Fric_rpm;
	uint32	Trigger_angle;
	uint8		Trigger_circle;
}Motor_Gimbal_Trigger_stc;

extern uint16_t Fric_pluse; 

extern Motor_Chassis_Rpm_stc  	Motor_Chassis_Rpm;
extern Motor_Gimbal_Yaw_stc  		Motor_Gimbal_Yaw;
extern Motor_Gimbal_Pitch_stc 	Motor_Gimbal_Pitch;
extern Motor_Gimbal_Trigger_stc	Motor_Gimbal_Trigger;


#define Gimbal_Yaw_Setangle	Motor_Gimbal_Yaw.Yaw_angle
#define Gimbal_Yaw_Setrpm		Motor_Gimbal_Yaw.Yaw_rpm
#define Gimbal_Pitch_Setangle	Motor_Gimbal_Pitch.Pitch_angle
#define Gimbal_Pitch_Setrpm		Motor_Gimbal_Pitch.Pitch_rpm
#define Gimbal_Trigger_Setcircle	Motor_Gimbal_Trigger.Trigger_circle
#define Gimbal_Trigger_set_angle	Motor_Gimbal_Trigger.Trigger_angle
#define Gimbal_Trigger_Setrpm		Motor_Gimbal_Trigger.Trigger_rpm
#define Gimbal_Fric_Setrpm	Motor_Gimbal_Trigger.Fric_rpm


enum Motor_Ctrlflag
{
	ChassisX = 1,
	ChassisY = 2,
	ChassisZ = 3,
	GimbalYaw = 4,
	GimbalPitch = 5,
	GimbalTrigger = 6,
	GimbalFric = 7,
};

void Motor_PID_init(void);		//初始化 电机PID计算结构体
void Gimbal_Ctrl(uint8_t motor_ID,float speed,uint16_t angle);	//云台电机控制函数
//void Fric_Ctrl(uint16_t ststus_flag);								//摩擦轮电机控制函数
void Chassis_Ctrl(uint8_t direct,float speed);		//底盘电机控制函数
void Motot_Control(void);			//电机控制主程序
void Urgency_Brake(void);			//紧急刹车 全部电机停转

#endif /* __MOTOR_H */
