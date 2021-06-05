/**
  ******************************************************************************
  * @file    Project/APP/angle_pid.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   pid 函数文件
  ******************************************************************************
  * @attention
  ******************************************************************************
*/

#include "angle_pid.h"
#include "motor.h"
#include <math.h>
#include "speed_pid.h"

#define aPID_OUT_MAX          30000		//即最大速度

// 函数: APID_Init()
// 描述: 电机机械角度pid参数初始化
// 参数：电机机械角度pid参数结构体
// 输出：无
// 内部函数，用户无需调用
void APID_Init(APID_t *apid)
{
	apid->actual_angle=0;
	apid->target_angle=0;
	apid->err=0;
	apid->last_err=0;
	apid->err_integration=0;
	apid->P_OUT=0;
	apid->I_OUT=0;
	apid->D_OUT=0;
	apid->PID_OUT=0;
}

// 函数: APID_Init_All()
// 描述: 所有电机机械角度pid参数初始化
// 参数：无
// 输出：无
void APID_Init_All()
{
	APID_Init(&motor1.apid);
	APID_Init(&motor2.apid);
	APID_Init(&motor3.apid);
	APID_Init(&motor4.apid);
  APID_Init(&motor5.apid);
	APID_Init(&motor6.apid);
}
void apid_realize(APID_t *apid,float kp,float ki,float kd)
{
	
	apid->err = apid->target_angle - apid->actual_angle;
	switch(switch_flag)
	{  
		case(HANDLE):
		{
			
    	apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
    }
		break;
	  case(TRIGGER):
		{
			apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
		}
		break;
		case(CHASSIC):
		{
			apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
			}
		break;
		default:break;
  }	
}

// 函数: apid_chassic_realize()
// 描述: 电机机械角度pid实现
// 参数：电机机械角度pid系数
// 输出：无
void apid_chassic_realize(float kp,float ki,float kd)
{
	//读取当前角度值
	motor1.apid.actual_angle = motor1.actual_angle;
	motor2.apid.actual_angle = motor2.actual_angle;
	motor3.apid.actual_angle = motor3.actual_angle;
	motor4.apid.actual_angle = motor4.actual_angle;
	
	switch_flag = CHASSIC;
	//计算电机机械角度pid
	apid_realize(&motor1.apid,kp,ki,kd);
	apid_realize(&motor2.apid,kp,ki,kd);
	apid_realize(&motor3.apid,kp,ki,kd);
	apid_realize(&motor4.apid,kp,ki,kd);
	switch_flag = NUL;

	//设置电机目标速度
	set_chassis_speed(motor1.apid.PID_OUT,motor2.apid.PID_OUT,motor3.apid.PID_OUT,motor4.apid.PID_OUT);
}

void apid_handle_realize(float kp,float ki,float kd)
{
	//读取当前角度值
	motor5.apid.actual_angle = motor5.total_angle-motor5.apid.trigger_first_total_angle_storage;
	motor6.apid.actual_angle = motor6.total_angle-motor6.apid.trigger_first_total_angle_storage;
	
	switch_flag = HANDLE;
	//计算电机机械角度pid
	apid_realize(&motor5.apid,kp,ki,kd);
	apid_realize(&motor6.apid,kp,ki,kd);
	switch_flag = NUL;
	
	set_handle_speed(motor5.apid.PID_OUT,motor6.apid.PID_OUT);
}

