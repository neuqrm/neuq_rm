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
#include "referee.h"

int errr_yaw[AVERAGE]={0};                           //云台yaw轴pid err数组
int err_pitch[AVERAGE]={0};                          //云台pitch轴pid
int average = AVERAGE;
PID_t pid_t;
enum switch_flag_t switch_flag; //不同模块的枚举类型

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
	vpid->pid_count=0;
	vpid->average_err=0;
	vpid->last_average_err=0; 
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
	VPID_Init(&gimbal_p.vpid);
	VPID_Init(&gimbal_y.vpid);
}
/**
  * @breif 速度环pid计算公式，微分项一般不需要
  * @attention 内部函数用户无需调用
  */

int flag_pitch=0,CHASSIS_vpid_max=9500;
int flag_yaw=0;
void vpid_realize(VPID_t *vpid,float kp,float ki,float kd)
{
		vpid->err = vpid->target_speed - vpid->actual_speed;
		
 switch(switch_flag)
 {
	 case(CHASSIS):
	 {
		if(abs(vpid->err) <= CHASSIS_IntegralSeparation)		//积分分离
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > CHASSIS_Integral_max)		//抗积分饱和
		vpid->err_integration = CHASSIS_Integral_max;
	  else if(vpid->err_integration < -CHASSIS_Integral_max)
		vpid->err_integration = -CHASSIS_Integral_max;
		
		vpid->P_OUT = kp * vpid->err;								//P项
	  vpid->I_OUT = ki * vpid->err_integration;		//I项
		
		//输出限幅
		if((vpid->P_OUT + vpid->I_OUT )> CHASSIS_vpid_max) 
		vpid->PID_OUT = CHASSIS_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -CHASSIS_vpid_max) 
		vpid->PID_OUT = -CHASSIS_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
   }
	 break;
   
   case(FIRE):
	 {
		if(abs(vpid->err) <= FIRE_IntegralSeparation)		//积分分离
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > FIRE_Integral_max)		//抗积分饱和
		vpid->err_integration = FIRE_Integral_max;
	  else if(vpid->err_integration < -FIRE_Integral_max)
		vpid->err_integration = -FIRE_Integral_max;
		
		vpid->P_OUT = kp * vpid->err;								//P项
	  vpid->I_OUT = ki * vpid->err_integration;		//I项
		
		//输出限幅
		if((vpid->P_OUT + vpid->I_OUT )> FIRE_vPID_max) 
		vpid->PID_OUT = CHASSIS_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -FIRE_vPID_max) 
		vpid->PID_OUT = -CHASSIS_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
   }
	 break;
   
	 case(TRIGGER):
	 {
	 	if(abs(vpid->err) <= TRIGGER_IntegralSeparation)		//积分分离
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > TRIGGER_Integral_max)		//抗积分饱和
		vpid->err_integration = TRIGGER_Integral_max;
	  else if(vpid->err_integration < -TRIGGER_Integral_max)
		vpid->err_integration = -TRIGGER_Integral_max;
		
		vpid->P_OUT = kp * vpid->err;								//P项
	  vpid->I_OUT = ki * vpid->err_integration;		//I项

		//输出限幅
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
		 
		 if(vpid->err <= GIMBAL_IntegralSeparation&& vpid->err >= -GIMBAL_IntegralSeparation)		//积分分离
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > GIMBAL_Integral_max)		//抗积分饱和
		vpid->err_integration = GIMBAL_Integral_max;
	  else if(vpid->err_integration < -GIMBAL_Integral_max)
		vpid->err_integration = -GIMBAL_Integral_max;
		
		
		vpid->P_OUT = kp * vpid->err;								//P项
	  vpid->I_OUT = ki * vpid->err_integration;		//I项
		vpid->D_OUT =kd * (vpid->err-vpid->last_err);
		vpid->last_err=vpid->err;

		//输出限幅
	  if((vpid->P_OUT + vpid->I_OUT+vpid->D_OUT)> GIMBAL_vPID_max) 
		vpid->PID_OUT = GIMBAL_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT+vpid->D_OUT ) < -GIMBAL_vPID_max) 
		vpid->PID_OUT = -GIMBAL_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT+vpid->D_OUT;
		
		
	 }
	  break;
	 	case(PITCH):
	 {
		if(abs(vpid->err) <= GIMBAL_IntegralSeparation)		//????
		vpid->err_integration += vpid->err;
	  if(vpid->err_integration > GIMBAL_Integral_max)		//?????
		vpid->err_integration = GIMBAL_Integral_max;
	  else if(vpid->err_integration < -GIMBAL_Integral_max)
		vpid->err_integration = -GIMBAL_Integral_max;		
		
		vpid->P_OUT = kp * vpid->err;								//P?
	   vpid->I_OUT = ki * vpid->err_integration;		//I?
		vpid->D_OUT =kd * (vpid->err-vpid->last_err);
		vpid->last_err=vpid->err;

		//????
	  if((vpid->P_OUT + vpid->I_OUT+vpid->D_OUT)> GIMBAL_vPID_max) 
		vpid->PID_OUT = GIMBAL_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT+vpid->D_OUT ) < -GIMBAL_vPID_max) 
		vpid->PID_OUT = -GIMBAL_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT+vpid->D_OUT;
	 }
	  break;

		default:break;
  }		
}

int limit_num=90;

/**
底盘功率限制
**/

void chassis_limit()
{
	if(Get_Chassis_Power()>=50)
		CHASSIS_vpid_max-=limit_num;
	else if(Get_Chassis_Power()<=40)
		CHASSIS_vpid_max+=limit_num;
	if(CHASSIS_vpid_max<=1300) CHASSIS_vpid_max=1300;
	else if(CHASSIS_vpid_max>=9000) CHASSIS_vpid_max=9000;
}


/**
  * @breif 底盘pid速度环运算函数
  */
void vpid_chassis_realize(float kp,float ki,float kd)
{
	//读取电机当前转速
	motor1.vpid.actual_speed = motor1.actual_speed;
	motor2.vpid.actual_speed = motor2.actual_speed;
	motor3.vpid.actual_speed = motor3.actual_speed;
	motor4.vpid.actual_speed = motor4.actual_speed;
	
	switch_flag = CHASSIS;
	
	//计算输出值
	
	//底盘功率限制的使用
	//chassis_limit();
	
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
  * @breif 拨弹轮pid速度环运算函数
  */
void vpid_trigger_realize(float kp,float ki,float kd)
{
	//读取电机当前转速
	motor5.vpid.actual_speed = motor5.actual_speed;
	switch_flag = TRIGGER;
	//计算输出值
	vpid_realize(&motor5.vpid,kp,ki,kd);
	switch_flag = NUL;
}
/**
  * @breif 云台pid速度环运算函数
  */
void vpid_gimbal_realize(float kp_y,float ki_y,float kd_y,float kp_p,float ki_p,float kd_p)
{
	//读取电机当前转速
	gimbal_y.vpid.actual_speed = gimbal_y.actual_speed;
	gimbal_p.vpid.actual_speed = gimbal_p.actual_speed;
	
	switch_flag = YAW;
	vpid_realize(&gimbal_y.vpid,kp_y,ki_y,kd_y);  //开始pid计算
	switch_flag = PITCH;
	vpid_realize(&gimbal_p.vpid,kp_p,ki_p,kd_p);
	switch_flag = NUL;
}
int fire1_actual_speed_record[12],fire2_actual_speed_record[12],actual_speed_sum_fire1=0,actual_speed_sum_fire2=0;//
int actual_speed_fire_count=0,actual_speed_fire_flag=0;	
int actual_fire1_actual_speed,actual_fire2_actual_speed;
/**
  * @breif 摩擦轮PID
  */
void vpid_fire_realize(float kp,float ki,float kd)
{
	//滤波除去异常值
	int max_fire1=-3000,min_fire1=3000,max_fire2=-3000,min_fire2=3000;	
	if(actual_speed_fire_count>=7) {actual_speed_fire_count=0;actual_speed_fire_flag=1;}
	actual_speed_sum_fire1-=fire1_actual_speed_record[actual_speed_fire_count];
	actual_speed_sum_fire2-=fire2_actual_speed_record[actual_speed_fire_count];
	fire1_actual_speed_record[actual_speed_fire_count]=fire1.actual_speed;
	fire2_actual_speed_record[actual_speed_fire_count]=fire2.actual_speed;
	actual_speed_sum_fire1+=fire1_actual_speed_record[actual_speed_fire_count];
	actual_speed_sum_fire2+=fire2_actual_speed_record[actual_speed_fire_count];
	actual_speed_fire_count++;
	for(int i=0;i<7;i++)
	{
		if(max_fire1<fire1_actual_speed_record[i]) max_fire1=fire1_actual_speed_record[i];
		if(min_fire1>fire1_actual_speed_record[i]) min_fire1=fire1_actual_speed_record[i];
		
		if(max_fire2<fire2_actual_speed_record[i]) max_fire2=fire2_actual_speed_record[i];
		if(min_fire2>fire2_actual_speed_record[i]) min_fire2=fire2_actual_speed_record[i];
	}
	if(actual_speed_fire_flag==0) 
	{
		int tt_;
		if((actual_speed_fire_count-2)<=0) tt_=1;
		else tt_=actual_speed_fire_count-2;
		actual_fire1_actual_speed=(actual_speed_sum_fire1-max_fire1-min_fire1)/tt_;
		actual_fire2_actual_speed=(actual_speed_sum_fire2-max_fire2-min_fire2)/tt_;
	}
	else
	{
		actual_fire1_actual_speed=(actual_speed_sum_fire1-max_fire1-min_fire1)/5;
		actual_fire2_actual_speed=(actual_speed_sum_fire2-max_fire2-min_fire2)/5;
	}
	//读取电机当前转速
	fire1.vpid.actual_speed = actual_fire1_actual_speed;
	fire2.vpid.actual_speed = actual_fire2_actual_speed;
	
	switch_flag = FIRE;
	
	//计算输出值
	
	vpid_realize(&fire1.vpid,kp,ki,kd);
	vpid_realize(&fire2.vpid,kp,ki,kd);
	
	switch_flag = NUL;
}

/************以下为测试用程序***************/
int pid_target_speed=0;//速度环测试变量
int pid_target_angle=4096;  //位置环测试变量

int pid_flag_start=1;
int pid_flag_end=0;
/**
  * @breif can模式下的云台测试程序，效果是手指拨动云台让其180度范围内来回旋转
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
  * @breif can模式下的云台测试程序
  */

int pid_pc(void)
{
	int a = 0;
a=4096-gimbal_y.actual_angle;
a=a*0.0347624*3;
  return a;
}
