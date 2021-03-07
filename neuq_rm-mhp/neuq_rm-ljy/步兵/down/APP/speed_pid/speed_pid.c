/**
  ******************************************************************************
  * @file    Project/APP/speed_pid.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   pid 函数文件
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include "speed_pid.h"
PID_t pid_t;
enum switch_flag_t switch_flag; 



void VPID_Init(VPID_t *vpid)
{
	vpid->target_speed=0;
	vpid->actual_speed=0;
	vpid->err=0;
	vpid->last_err=0;
	vpid->err_integration=0;
	vpid->P_OUT=0;
	vpid->I_OUT=0;
	vpid->D_OUT=0;
	vpid->PID_OUT=0;
}



void VPID_Init_All()	
{
	VPID_Init(&motor1.vpid);
	VPID_Init(&motor2.vpid);
	VPID_Init(&motor3.vpid);
	VPID_Init(&motor4.vpid);
	VPID_Init(&motor5.vpid);
	VPID_Init(&gimbal_p.vpid);
	VPID_Init(&gimbal_y.vpid);
}



void vpid_realize(VPID_t *vpid,float kp,float ki,float kd)
{
	vpid->err = vpid->target_speed - vpid->actual_speed;
	switch(switch_flag)
	{
		case(CHASSIC):
		{
			if(abs(vpid->err) <= CHASSIC_IntegralSeparation)		
				vpid->err_integration += vpid->err;
			if(vpid->err_integration > CHASSIC_Integral_max)		
				vpid->err_integration = CHASSIC_Integral_max;
			else if(vpid->err_integration < -CHASSIC_Integral_max)
				vpid->err_integration = -CHASSIC_Integral_max;
			vpid->P_OUT = kp * vpid->err;								
			vpid->I_OUT = ki * vpid->err_integration;		
			if((vpid->P_OUT + vpid->I_OUT )> CHASSIC_vPID_max) 
				vpid->PID_OUT = CHASSIC_vPID_max;
			else if((vpid->P_OUT + vpid->I_OUT ) < -CHASSIC_vPID_max) 
				vpid->PID_OUT = -CHASSIC_vPID_max;
			else
				vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
		}
		break;
		case(TRIGGER):
		{
			if(abs(vpid->err) <= TRIGGER_IntegralSeparation)		
				vpid->err_integration += vpid->err;
			if(vpid->err_integration > TRIGGER_Integral_max)		
				vpid->err_integration = TRIGGER_Integral_max;
			else if(vpid->err_integration < -TRIGGER_Integral_max)
				vpid->err_integration = -TRIGGER_Integral_max;
			vpid->P_OUT = kp * vpid->err;								
			vpid->I_OUT = ki * vpid->err_integration;		
			if((vpid->P_OUT + vpid->I_OUT )> TRIGGER_vPID_max) 
				vpid->PID_OUT = TRIGGER_vPID_max;
			else if((vpid->P_OUT + vpid->I_OUT ) < -TRIGGER_vPID_max) 
				vpid->PID_OUT = -TRIGGER_vPID_max;
			else
				vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
		}
		break;
		case(GIMBAL):
		{
			if(abs(vpid->err) <= GIMBAL_IntegralSeparation)		//积分分离
				vpid->err_integration += vpid->err;
			if(vpid->err_integration > GIMBAL_Integral_max)		//抗积分饱和
				vpid->err_integration = GIMBAL_Integral_max;
			else if(vpid->err_integration < -GIMBAL_Integral_max)
				vpid->err_integration = -GIMBAL_Integral_max;
	
			vpid->P_OUT = kp * vpid->err;								//P项
			vpid->I_OUT = ki * vpid->err_integration;		//I项
			vpid->D_OUT = kd * (vpid->err-vpid->last_err);//D项
			vpid->last_err=vpid->err;
	  //输出限幅
			if(abs(vpid->err) <= 2)
				vpid->PID_OUT=0;
			else
			{	
				if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT)> GIMBAL_vPID_max) 
					vpid->PID_OUT = GIMBAL_vPID_max;
				else if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT) < -GIMBAL_vPID_max) 
					vpid->PID_OUT = -GIMBAL_vPID_max;
				else
					vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT + vpid->D_OUT;
			}
		}	
		break;
		default:break;
	}		
}



void vpid_chassic_realize(float kp,float ki,float kd)
{
	motor1.vpid.actual_speed = motor1.actual_speed;
	motor2.vpid.actual_speed = motor2.actual_speed;
	motor3.vpid.actual_speed = motor3.actual_speed;
	motor4.vpid.actual_speed = motor4.actual_speed;
	switch_flag = CHASSIC;
	vpid_realize(&motor1.vpid,kp,ki,kd);
	vpid_realize(&motor2.vpid,kp,ki,kd);
	vpid_realize(&motor3.vpid,kp,ki,kd);
	vpid_realize(&motor4.vpid,kp,ki,kd);
	switch_flag = NUL;
}



void vpid_trigger_realize(float kp,float ki,float kd)
{
	motor5.vpid.actual_speed = motor5.actual_speed;
	switch_flag = TRIGGER;
	vpid_realize(&motor5.vpid,kp,ki,kd);
	switch_flag = NUL;
}



void vpid_gimbal_realize(float kp_y,float ki_y,float kd_y,float kp_p,float ki_p,float kd_p)
{
	gimbal_y.vpid.actual_speed = gimbal_y.actual_speed;
	gimbal_y.vpid.actual_speed = gimbal_y.actual_speed;
	switch_flag = GIMBAL;
	vpid_realize(&gimbal_y.vpid,kp_y,ki_y,kd_y);  
	vpid_realize(&gimbal_p.vpid,kp_p,ki_p,kd_p);
	switch_flag = NUL;
}



int pid_target_speed=0;
int pid_target_angle=4096;  
int pid_flag_start=1;
int pid_flag_end=0;



int pid_auto(void)
{
	int a=0;
   if(gimbal_y.actual_angle>=2048&&gimbal_y.actual_angle<=6144&&pid_flag_start)
	 {
	   a=60;
	 }
	 if(gimbal_y.actual_angle>=2048&&gimbal_y.actual_angle<=6144&&pid_flag_end)
	 {
		 a=-60;
	 }
   if(gimbal_y.actual_angle<2048&&gimbal_y.actual_angle>2008)
	 {
	 a=0;
	 }
	 if(gimbal_y.actual_angle<6184&&gimbal_y.actual_angle>6144)
	 {
	 a=0;
	 }

	 if(gimbal_y.actual_angle>=6184)
	 {
	   a=0;
		 pid_flag_start=0;
		 pid_flag_end=1;
	 }
	 if(gimbal_y.actual_angle<=2008)
	 {
	  a=0;
		pid_flag_start=1;
		pid_flag_end=0;
	 } 
	 return a;
}



int pid_pc(void)
{
	int a = 0;
a=4096-gimbal_y.actual_angle;
a=a*0.0347624*3;
  return a;
}
