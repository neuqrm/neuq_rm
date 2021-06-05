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
//int vpid_out_max=vPID_OUT_MAX;
enum switch_flag_t switch_flag;

/**
  * @breif 电机转速pid参数初始化
	* @attention 内部函数，无需调用
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
}

/**
  * @breif 电机速度环pid初始化
  */
void VPID_Init_All()	
{
	VPID_Init(&motor1.vpid);
	VPID_Init(&motor2.vpid);
	VPID_Init(&motor3.vpid);
	VPID_Init(&motor4.vpid);
	VPID_Init(&motor5.vpid);
	VPID_Init(&motor6.vpid);
}


/**
  * @breif 速度环pid计算公式，微分项一般不需要
  * @attention 内部函数用户无需调用
  */
void vpid_realize(VPID_t *vpid,float kp,float ki,float kd)
{
		vpid->err = vpid->target_speed - vpid->actual_speed;
		
 switch(switch_flag)
 {
	 case(CHASSIC):
	 {
		if(abs(vpid->err) <= CHASSIC_IntegralSeparation)		//积分分离
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > CHASSIC_Integral_max)		//抗积分饱和
		vpid->err_integration = CHASSIC_Integral_max;
	  else if(vpid->err_integration < -CHASSIC_Integral_max)
		vpid->err_integration = -CHASSIC_Integral_max;
		
		vpid->P_OUT = kp * vpid->err;								//P项
	  vpid->I_OUT = ki * vpid->err_integration;		//I项
		
		//输出限幅
		if((vpid->P_OUT + vpid->I_OUT )> CHASSIC_vPID_max) 
		vpid->PID_OUT = CHASSIC_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -CHASSIC_vPID_max) 
		vpid->PID_OUT = -CHASSIC_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
   }
	 break;
	 case(HANDLE):
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

/**
  * @breif 底盘pid速度环运算函数
  */
void vpid_chassic_realize(float kp,float ki,float kd)
{
	//读取电机当前转速
	motor1.vpid.actual_speed = motor1.actual_speed;
	motor2.vpid.actual_speed = motor2.actual_speed;
	motor3.vpid.actual_speed = motor3.actual_speed;
	motor4.vpid.actual_speed = motor4.actual_speed;
	
	switch_flag = CHASSIC;
	//计算输出值
	vpid_realize(&motor1.vpid,kp,ki,kd);
	vpid_realize(&motor2.vpid,kp,ki,kd);
	vpid_realize(&motor3.vpid,kp,ki,kd);
	vpid_realize(&motor4.vpid,kp,ki,kd);
	
	switch_flag = NUL;
//功率控制方案（测试）
/*	power_limitation_jugement();
	power_limitation_coefficient();*/
}
/**
  * @breif 抓手pid速度环运算函数
  */
void vpid_handle_realize(float kp,float ki,float kd)
{
	//读取电机当前转速
	motor5.vpid.actual_speed = motor5.actual_speed;
	motor6.vpid.actual_speed = motor6.actual_speed;
	
	switch_flag = HANDLE;
	
	vpid_realize(&motor5.vpid,kp,ki,kd);  //开始pid计算
	//vpid_realize(&motor6.vpid,kp,ki,kd);
	
	switch_flag = NUL;
}

/************以下为测试用程序***************/
int pid_target_speed=0;//速度环测试变量
int pid_target_angle=4096;  //位置环测试变量

