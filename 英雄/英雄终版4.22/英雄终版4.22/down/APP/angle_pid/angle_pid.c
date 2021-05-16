/**
  ******************************************************************************
  * @file    Project/APP/angle_pid.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   pid �����ļ�
  ******************************************************************************
  * @attention
  ******************************************************************************
*/

#include "angle_pid.h"
#include "motor.h"
#include <math.h>
#include "speed_pid.h"
#include "remote_code.h"

// ����: APID_Init()
// ����: �����е�Ƕ�pid������ʼ��
// �����������е�Ƕ�pid�����ṹ��
// �������
// �ڲ��������û��������
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

// ����: APID_Init_All()
// ����: ���е����е�Ƕ�pid������ʼ��
// ��������
// �������
void APID_Init_All()
{
	APID_Init(&motor1.apid);
	APID_Init(&motor2.apid);
	APID_Init(&motor3.apid);
	APID_Init(&motor4.apid);
    APID_Init(&motor5.apid);
	
	APID_Init(&gimbal_y.apid);
	APID_Init(&gimbal_p.apid);
	//gimbal_p.apid.target_angle=-120;
}
void apid_realize(APID_t *apid,float kp,float ki,float kd)
{
	
	apid->err = apid->target_angle - apid->actual_angle;
	switch(switch_flag)
	{  
		case(YAW):
		{
			 if(apid->err <= GIMBAL_aIntegralSeparation&& apid->err >= -GIMBAL_aIntegralSeparation)		//���ַ���
			apid->err_integration += apid->err;
	  if(apid->err_integration > GIMBAL_Integral_amax)		//�����ֱ���
		apid->err_integration = GIMBAL_Integral_amax;
	  else if(apid->err_integration < -GIMBAL_Integral_amax)
		apid->err_integration = -GIMBAL_Integral_amax;
			
			if(apid->err<=-PI)
				apid->err+=PI2;
			if(apid->err>=PI)
				apid->err-=PI2;
			
    	apid->P_OUT = kp * apid->err;
			apid->I_OUT = ki * apid->err_integration;		//I��
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT+apid->I_OUT) > aPID_OUT_MAX_YAW) 			
	   	apid->PID_OUT = aPID_OUT_MAX_YAW;
	    else if((apid->P_OUT + apid->D_OUT+apid->I_OUT) < -aPID_OUT_MAX_YAW) 
		  apid->PID_OUT = -aPID_OUT_MAX_YAW;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT+apid->I_OUT;
		
		if(abs(apid->D_OUT)>abs(apid->P_OUT)) apid->PID_OUT=0;
    }
		break;
		case(PITCH):
		{
			if(apid->err <= 20&& apid->err >= -20)		//���ַ���
			apid->err_integration += apid->err;
	  if(apid->err_integration > 50)		//�����ֱ���
		apid->err_integration = 50;
	  else if(apid->err_integration < -50)
		apid->err_integration = -50;
		
    	apid->P_OUT = kp * apid->err;
		  apid->I_OUT = ki * apid->err_integration;		//I��
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT+apid->I_OUT) > a_PITCH_PID_OUT_MAX) 			
	   	apid->PID_OUT = a_PITCH_PID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT+apid->I_OUT) < -a_PITCH_PID_OUT_MAX) 
		  apid->PID_OUT = -a_PITCH_PID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT+apid->I_OUT;
		if(abs(apid->D_OUT)>abs(apid->P_OUT)) apid->PID_OUT=0;
    }
		break;

	  case(TRIGGER):
		{
			apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > a_TRIGGER_PID_OUT_MAX) 			
	   	apid->PID_OUT = a_TRIGGER_PID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -a_TRIGGER_PID_OUT_MAX) 
		  apid->PID_OUT = -a_TRIGGER_PID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
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
		default:break;
  }	
}

// ����: apid_chassis_realize()
// ����: �����е�Ƕ�pidʵ��
// �����������е�Ƕ�pidϵ��
// �������
void apid_chassis_realize(float kp,float ki,float kd)
{
	//��ȡ��ǰ�Ƕ�ֵ
	motor1.apid.actual_angle = motor1.total_angle;
	motor2.apid.actual_angle = motor2.total_angle;
	motor3.apid.actual_angle = motor3.total_angle;
	motor4.apid.actual_angle = motor4.total_angle;
	
	switch_flag = CHASSIS;
	//��������е�Ƕ�pid
	apid_realize(&motor1.apid,kp,ki,kd);
	apid_realize(&motor2.apid,kp,ki,kd);
	apid_realize(&motor3.apid,kp,ki,kd);
	apid_realize(&motor4.apid,kp,ki,kd);
	switch_flag = NUL;

	//���õ��Ŀ���ٶ�
	set_chassis_speed(motor1.apid.PID_OUT,motor2.apid.PID_OUT,motor3.apid.PID_OUT,motor4.apid.PID_OUT);
}
extern float theta,alpha;
void apid_gimbal_realize(float kp_y,float ki_y,float kd_y,float kp_p,float ki_p,float kd_p)
{
	//��ȡ��ǰ�Ƕ�ֵ
	//gimbal_y.apid.actual_angle = gimbal_y.actual_angle;
	//theta=Get_gimbal_theta();
	
	gimbal_y.apid.actual_angle = alpha+theta;   
  if(gimbal_y.apid.actual_angle>=2*PI)	gimbal_y.apid.actual_angle=gimbal_y.apid.actual_angle-2*PI;//С����
	
	
	gimbal_p.apid.actual_angle = gimbal_p.actual_angle;
	
	switch_flag = YAW;                                //�жϱ�־λ
	//��������е�Ƕ�pid
	apid_realize(&gimbal_y.apid,kp_y,ki_y,kd_y);
	switch_flag = PITCH;                                //�жϱ�־λ
	apid_realize(&gimbal_p.apid,kp_p,ki_p,kd_p);
	switch_flag = NUL;                                   //��־λ����
	//if(gimbal_y.apid.actual_angle>=7560) gimbal_y.apid.PID_OUT=0;
	//if(gimbal_y.apid.actual_angle<=10)   gimbal_y.apid.PID_OUT=0;
	set_gimbal_speed(gimbal_y.apid.PID_OUT,gimbal_p.apid.PID_OUT);
}


void apid_trigeer_realize(float kp,float ki,float kd)
{
	motor5.apid.actual_angle = motor5.total_angle-motor5.start_angle;//
	switch_flag = TRIGGER;
	apid_realize(&motor5.apid,kp,ki,kd);
	switch_flag = NUL;
	set_trigger_speed(motor5.apid.PID_OUT);
}
