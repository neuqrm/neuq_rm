/**
  ******************************************************************************
  * @file    Project/APP/angle_pid.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   pid º¯ÊýÎÄ¼þ
  ******************************************************************************
  * @attention
  ******************************************************************************
*/

#include "angle_pid.h"
#include "motor.h"
#include <math.h>
#include "speed_pid.h"

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


void APID_Init_All()
{
	APID_Init(&motor1.apid);
	APID_Init(&motor2.apid);
	APID_Init(&motor3.apid);
	APID_Init(&motor4.apid);
	APID_Init(&motor5.apid);
	APID_Init(&gimbal_y.apid);
	APID_Init(&gimbal_p.apid);
}
void apid_realize(APID_t *apid,float kp,float ki,float kd)
{
	
	apid->err = apid->target_angle - apid->actual_angle;
	apid->err = 0.5*(apid->err +apid->last_err);
	switch(switch_flag)
	{  
		case(YAW):
		{	
			if(abs(apid->err) <= yaw_i_seperation)		
				apid->err_integration += apid->err;
			if(apid->err_integration > yaw_i_max)		
				apid->err_integration = yaw_i_max;
			else if(apid->err_integration < -yaw_i_max)
				apid->err_integration = -yaw_i_max;

			apid->P_OUT = kp * apid->err;
			apid->I_OUT = ki * apid->err_integration;
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
		case(PITCH):
		{
			if(abs(apid->err) <= yaw_i_seperation)		
				apid->err_integration += apid->err;
			if(apid->err_integration > yaw_i_max)		
				apid->err_integration = yaw_i_max;
			else if(apid->err_integration < -yaw_i_max)
				apid->err_integration = -yaw_i_max;

			apid->P_OUT = kp * apid->err;
			apid->I_OUT = ki * apid->err_integration;
			apid->D_OUT = kd * (apid->err-apid->last_err);
			apid->last_err = apid->err;

			if((apid->P_OUT + apid->D_OUT +apid->I_OUT) > a_PITCH_PID_OUT_MAX) 			
				apid->PID_OUT = a_PITCH_PID_OUT_MAX;
			else if((apid->P_OUT + apid->D_OUT +apid->I_OUT) < -a_PITCH_PID_OUT_MAX) 
				apid->PID_OUT = -a_PITCH_PID_OUT_MAX;
			else
				apid->PID_OUT = apid->P_OUT + apid->D_OUT +apid->I_OUT;
		}
		break;
		case(TRIGGER):
		{
			apid->err = apid->target_angle - apid->actual_angle;//mhp
			apid->P_OUT = kp * apid->err;
			apid->D_OUT = kd * (apid->err-apid->last_err);
			apid->last_err = apid->err;
			if((apid->P_OUT + apid->D_OUT) > TRIGGER_PID_OUT_MAX)
				apid->PID_OUT = TRIGGER_PID_OUT_MAX;
			else if((apid->P_OUT + apid->D_OUT) < -TRIGGER_PID_OUT_MAX) 
				apid->PID_OUT = -TRIGGER_PID_OUT_MAX;
			else
				apid->PID_OUT =(apid->P_OUT + apid->D_OUT);
		}
		break;
		case(CHASSIS):
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
		default:
			break;
	}	
}

void apid_chassis_realize(float kp,float ki,float kd)
{
	motor1.apid.actual_angle = motor1.total_angle;
	motor2.apid.actual_angle = motor2.total_angle;
	motor3.apid.actual_angle = motor3.total_angle;
	motor4.apid.actual_angle = motor4.total_angle;
	switch_flag = CHASSIS;
	apid_realize(&motor1.apid,kp,ki,kd);
	apid_realize(&motor2.apid,kp,ki,kd);
	apid_realize(&motor3.apid,kp,ki,kd);
	apid_realize(&motor4.apid,kp,ki,kd);
	switch_flag = NUL;
	set_chassis_speed(motor1.apid.PID_OUT,motor2.apid.PID_OUT,motor3.apid.PID_OUT,motor4.apid.PID_OUT);
}

void apid_gimbal_realize(float kp_y,float ki_y,float kd_y,float kp_p,float ki_p,float kd_p)
{
	gimbal_y.apid.actual_angle = gimbal_y.actual_angle;
	gimbal_p.apid.actual_angle = gimbal_p.actual_angle;
	switch_flag = YAW;                                
	apid_realize(&gimbal_y.apid,kp_y,ki_y,kd_y);
	switch_flag = PITCH;                                
	apid_realize(&gimbal_p.apid,kp_p,ki_p,kd_p);
	switch_flag = NUL;                                  
	set_gimbal_speed(gimbal_y.apid.PID_OUT,gimbal_p.apid.PID_OUT);
}
//*************************************************************************mhp111

void apid_trigeer_realize(float kp,float ki,float kd)
{
	motor5.apid.actual_angle = motor5.total_angle-motor5.apid.trigger_first_total_angle_storage;
	switch_flag = TRIGGER;
	apid_realize(&motor5.apid,kp,ki,kd);
	switch_flag = NUL;
	set_trigger_speed(motor5.apid.PID_OUT);
}//mhp222
