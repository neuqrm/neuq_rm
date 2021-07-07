#ifndef __MOTION_H
#define	__MOTION_H

#include "bsp_dbus.h"
#include "remote.h"
#include "motor.h"
#include "math.h"
#include "imu.h"
#include "json.h"
#include "basicdata.h"
#include "CAN_receive.h"


typedef struct  
{
	float Xrpm;
	float Yrpm;
	float Zrpm;
	float ZAngle;
	float ZreAngle;
}Motion_Chassis_Span_Stc;

extern Motion_Chassis_Span_Stc	Chassis_Span_Stc;
extern Chassis_Rpm_stc  	Motion_Chassis_Rpm;
extern Gimbal_Yaw_stc  		Motion_Gimbal_Yaw;
extern Gimbal_Pitch_stc 	Motion_Gimbal_Pitch;

extern void Motion_Ctrl(void);
extern void Motion_Gimbal_Ctrl(void);
extern void Motion_Chassis_Ctrl(void);
extern void Motion_Shoot_Ctrl(void);
extern void Span_Move_Chassis(float ucAngle);
extern void Span_Move_Gimbal(float ucAngle);
extern void Yaw_Angle_Calculate(float ucAngle);
extern void Span_Move_Calculate(float ucAngle);
extern void Singal_Shoot_Calculate(void);

#endif /*__MOTION_H*/
