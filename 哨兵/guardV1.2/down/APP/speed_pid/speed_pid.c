/**
  ******************************************************************************
  * @file    Project/APP/speed_pid.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   pid �����ļ�
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include "speed_pid.h"

int errr_yaw[AVERAGE]={0};                           //��̨yaw��pid err����
int err_pitch[AVERAGE]={0};                          //��̨pitch��pid
int average = AVERAGE;
PID_t pid_t;
enum switch_flag_t switch_flag; //��ͬģ���ö������

/**
  * @breif ���ת��pid������ʼ��
	* @attention �ڲ��������������
	*/
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
	vpid->pid_count=0;
	vpid->average_err=0;
	vpid->last_average_err=0; 
}

/**
  * @breif ����ٶȻ�pid��ʼ��
  */
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
/**
  * @breif �ٶȻ�pid���㹫ʽ��΢����һ�㲻��Ҫ
  * @attention �ڲ������û��������
  */
void vpid_realize(VPID_t *vpid,float kp,float ki,float kd)
{
		vpid->err = vpid->target_speed - vpid->actual_speed;
		
 switch(switch_flag)
 {
	 case(CHASSIS):
	 {
		if(abs(vpid->err) <= CHASSIS_IntegralSeparation)		//���ַ���
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > CHASSIS_Integral_max)		//�����ֱ���
		vpid->err_integration = CHASSIS_Integral_max;
	  else if(vpid->err_integration < -CHASSIS_Integral_max)
		vpid->err_integration = -CHASSIS_Integral_max;
		
		vpid->P_OUT = kp * vpid->err;								//P��
	  vpid->I_OUT = ki * vpid->err_integration;		//I��
		
		//����޷�
		if((vpid->P_OUT + vpid->I_OUT )> CHASSIS_vPID_max) 
		vpid->PID_OUT = CHASSIS_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -CHASSIS_vPID_max) 
		vpid->PID_OUT = -CHASSIS_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
   }
	 break;
	 case(TRIGGER):
	 {
	 	if(abs(vpid->err) <= TRIGGER_IntegralSeparation)		//���ַ���
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > TRIGGER_Integral_max)		//�����ֱ���
		vpid->err_integration = TRIGGER_Integral_max;
	  else if(vpid->err_integration < -TRIGGER_Integral_max)
		vpid->err_integration = -TRIGGER_Integral_max;
		
		vpid->P_OUT = kp * vpid->err;								//P��
	  vpid->I_OUT = ki * vpid->err_integration;		//I��

		//����޷�
	  if((vpid->P_OUT + vpid->I_OUT )> TRIGGER_vPID_max) 
		vpid->PID_OUT = TRIGGER_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -TRIGGER_vPID_max) 
		vpid->PID_OUT = -TRIGGER_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
	 }
		break;
	 case(YAW):
	 {
		if(vpid->pid_count>=average)
		vpid->pid_count=0;
		errr_yaw[vpid->pid_count] = 0.5*(vpid->err+vpid->last_err);
		vpid->pid_count++;
		for(int i=0;i<=(average-1);i++)
		vpid->average_err += (errr_yaw[i]);
	  	
	 	if(abs(vpid->err) <= GIMBAL_IntegralSeparation)		//���ַ���
		vpid->err_integration += vpid->average_err;
	  if(vpid->err_integration > GIMBAL_Integral_max)		//�����ֱ���
		vpid->err_integration = GIMBAL_Integral_max;
	  else if(vpid->err_integration < -GIMBAL_Integral_max)
		vpid->err_integration = -GIMBAL_Integral_max;
		
		
	  vpid->P_OUT = 0.1f*kp*vpid->average_err;								//P��
	  vpid->I_OUT = ki * vpid->err_integration;		//I��
	  vpid->D_OUT = kd * (vpid->average_err-vpid->last_average_err);//D��
	  vpid->last_err=vpid->err;
		vpid->last_average_err =  vpid->average_err;
	  //����޷� 
	  if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT)> GIMBAL_vPID_max) 
		vpid->PID_OUT = GIMBAL_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT) < -GIMBAL_vPID_max) 
		vpid->PID_OUT = -GIMBAL_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT + vpid->D_OUT;
		
		errr_yaw[(vpid->pid_count-1)] = 0;
	 }
	  break;
	 	case(PITCH):
	 {
		if(vpid->pid_count>=average)
		vpid->pid_count=0;
		err_pitch[vpid->pid_count] = 0.5*(vpid->err+vpid->last_err);
		vpid->pid_count++;
		for(int i=0;i<=(average-1);i++)
		vpid->average_err += (err_pitch[i]);
	  	
	 	if(abs(vpid->err) <= GIMBAL_IntegralSeparation)		//���ַ���
		vpid->err_integration += vpid->average_err;
	  if(vpid->err_integration > GIMBAL_Integral_max)		//�����ֱ���
		vpid->err_integration = GIMBAL_Integral_max;
	  else if(vpid->err_integration < -GIMBAL_Integral_max)
		vpid->err_integration = -GIMBAL_Integral_max;
		
	  vpid->P_OUT = 0.1f*kp*vpid->average_err;								//P��
	  vpid->I_OUT = ki * vpid->err_integration;		//I��
	  vpid->D_OUT = kd * (vpid->average_err-vpid->last_average_err);//D��
	  vpid->last_err=vpid->err;
		vpid->last_average_err =  vpid->average_err;
	  //����޷� 
	  if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT)> GIMBAL_vPID_max) 
		vpid->PID_OUT = GIMBAL_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT) < -GIMBAL_vPID_max) 
		vpid->PID_OUT = -GIMBAL_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT + vpid->D_OUT;
		
		err_pitch[(vpid->pid_count-1)] = 0;
	 }
	  break;

		default:break;
  }		
}

/**
  * @breif ����pid�ٶȻ����㺯��
  */
void vpid_chassis_realize(float kp,float ki,float kd)
{
	//��ȡ�����ǰת��
	motor1.vpid.actual_speed = motor1.actual_speed;
	motor2.vpid.actual_speed = motor2.actual_speed;
	motor3.vpid.actual_speed = motor3.actual_speed;
	motor4.vpid.actual_speed = motor4.actual_speed;
	
	switch_flag = CHASSIS;
	//�������ֵ
	vpid_realize(&motor1.vpid,kp,ki,kd);
	vpid_realize(&motor2.vpid,kp,ki,kd);
	vpid_realize(&motor3.vpid,kp,ki,kd);
	vpid_realize(&motor4.vpid,kp,ki,kd);
	
	switch_flag = NUL;
//���ʿ��Ʒ��������ԣ�
/*	power_limitation_jugement();
	power_limitation_coefficient();*/
}
/**
  * @breif ������pid�ٶȻ����㺯��
  */
void vpid_trigger_realize(float kp,float ki,float kd)
{
	//��ȡ�����ǰת��
	motor5.vpid.actual_speed = motor5.actual_speed;
	switch_flag = TRIGGER;
	//�������ֵ
	vpid_realize(&motor5.vpid,kp,ki,kd);
	switch_flag = NUL;
}
/**
  * @breif ��̨pid�ٶȻ����㺯��
  */
void vpid_gimbal_realize(float kp_y,float ki_y,float kd_y,float kp_p,float ki_p,float kd_p)
{
	//��ȡ�����ǰת��
	gimbal_y.vpid.actual_speed = gimbal_y.actual_speed;
	gimbal_p.vpid.actual_speed = gimbal_p.actual_speed;
	
	switch_flag = YAW;
	vpid_realize(&gimbal_y.vpid,kp_y,ki_y,kd_y);  //��ʼpid����
	switch_flag = PITCH;
	vpid_realize(&gimbal_p.vpid,kp_p,ki_p,kd_p);
	switch_flag = NUL;
}

/************����Ϊ�����ó���***************/
int pid_target_speed=0;//�ٶȻ����Ա���
int pid_target_angle=4096;  //λ�û����Ա���

int pid_flag_start=1;
int pid_flag_end=0;
/**
  * @breif canģʽ�µ���̨���Գ���Ч������ָ������̨����180�ȷ�Χ��������ת
  */
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
/**
  * @breif canģʽ�µ���̨���Գ���
  */

int pid_pc(void)
{
	int a = 0;
a=4096-gimbal_y.actual_angle;
a=a*0.0347624*3;
  return a;
}
