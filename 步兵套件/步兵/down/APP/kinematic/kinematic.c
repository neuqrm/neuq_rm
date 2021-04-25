/**
  ******************************************************************************
  * @file    Project/APP/kinematic.c 
  * @author  Siyuan Qiao&Junyu Luo 
  * @version V1.0.0
  * @date    2.2020
  * @brief   底盘正逆运动学演算
  *          线速度单位： cm/s
  *          角速度单位： rad/s
	*          转速单位：   rpm
  ******************************************************************************
  * @attention
  ******************************************************************************
  */


#include "kinematic.h"
#include "motor.h"
#include "speed_pid.h"
#include "algorithm.h"
#include "gimbal.h"
#include "shoot.h"
#include "power_limitation.h"
Kinematics_t Kinematics;

//逆运动学公式
//把想要得到的底盘速度转换为轮子的线速度
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.wheel1.target_speed.linear_vel = linear_x - linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel2.target_speed.linear_vel = linear_x + linear_y - angular_z*(half_width+half_length);
	Kinematics.wheel3.target_speed.linear_vel = linear_x + linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel4.target_speed.linear_vel = linear_x -  linear_y - angular_z*(half_width+half_length);
	//线速度 cm/s  转转度  RPM 
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel4.target_speed.rpm = Kinematics.wheel4.target_speed.linear_vel * VEL2RPM;
	
	motor1.target_speed = - (int)(Kinematics.wheel1.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor2.target_speed =   (int)(Kinematics.wheel2.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor3.target_speed =  - (int)(Kinematics.wheel3.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor4.target_speed =  (int)(Kinematics.wheel4.target_speed.rpm * M3508_REDUCTION_RATIO);
	
}



//正运动学公式
//通过轮胎的实际转速计算底盘几何中心的三轴速度
void Get_Base_Velocities(void)
{
	//根据电机转速测算轮子转速
	Kinematics.wheel1.actual_speed.rpm = - motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm =   motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel3.actual_speed.rpm =   motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel4.actual_speed.rpm = - motor4.actual_speed / M3508_REDUCTION_RATIO;
	//轮子转速转换为轮子线速度
	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel4.actual_speed.linear_vel = Kinematics.wheel4.actual_speed.rpm * RPM2VEL;
	//轮子线速度转换为底盘中心三轴的速度
	Kinematics.actual_velocities.angular_z = ( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f*(half_width + half_length));
	Kinematics.actual_velocities.linear_x  = (-Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
	Kinematics.actual_velocities.linear_y  = ( Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				+ Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
}

void Get_Gimbal_Angle()
{
  Kinematics.yaw.actual_angle = (gimbal_y.actual_angle-BASIC_YAW_ANGLE_CAN)*(360.0f/GM6020_ENCODER_ANGLE);
  Kinematics.pitch.actual_angle = (gimbal_p.actual_angle-BASIC_PITCH_ANGLE_CAN)*(360.0f/GM6020_ENCODER_ANGLE);
}

// 函数: speed_control()
// 描述: 将pid速度输出转换为电机速度，最终传递给速度pid
// 参数：三个方向的速度
// 输出：4个电机速度
// 注：电机1、4的默认旋转方向和车轮实际正方向相反，需要取反
int find_max(void);
int stop_flag_chassis=0;

void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
	/*if(stop_flag_chassis == 0 && speed_x == 0 && speed_y == 0 && speed_r == 0)
	{
		stop_flag_chassis = 1;			//停止   此标志为了避免多次进入
		stop_chassis_motor();			//停下来  并角度闭环
	}
	else if(speed_x != 0 || speed_y != 0 || speed_r != 0)
	{		*/
	  max=find_max();

		stop_flag_chassis = 0;
		//速度换算
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
	 
    power_limitation_jugement();    //功率控制判断函数
		
		if(max>max_motor_speed)
		{
			motor1.target_speed=(int)(motor1.target_speed*max_motor_speed*1.0/max);
			motor2.target_speed=(int)(motor2.target_speed*max_motor_speed*1.0/max);
			motor3.target_speed=(int)(motor3.target_speed*max_motor_speed*1.0/max);
			motor4.target_speed=(int)(motor4.target_speed*max_motor_speed*1.0/max);
		}
		  //power_limitation_scale();
			//改变速度pid目标速度
			set_chassis_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
	
}	

int stop_flag_trigger=0;

void trigger_control(float trigger_angular)
{
if(stop_flag_trigger == 0 && trigger_angular==0)
	{
		stop_flag_trigger = 1;			//停止   此标志为了避免多次进入
		stop_trigger_motor();			//停下来  并角度闭环
	}
else if(trigger_angular!=0)
	{
		stop_flag_trigger = 0;
		
		trigger_to_motor(trigger_angular);
		
		set_trigger_speed(motor5.target_speed);		
}
	}

int flag_1=1;
	
void gimbal_speed_control(float gimbal_y_speed,float gimbal_p_speed)    //
{
	
	 /*	if(gimbal_y_speed == 0)
	{
		stop_gimbal_motor();			//停下来  并角度闭环
	}
    else*/
    //gimbal_y_speed = KalmanFilter(gimbal_y_speed,1,200);
	/*  if(Kinematics.pitch.actual_angle<=-45)
			flag_1=0;
		if(Kinematics.pitch.actual_angle>45)
			flag_1=1;
		if(flag_1==0)
		{
		Kinematics.pitch.target_angular=0;
		gimbal_p.vpid.PID_OUT=0;
		gimbal_p.vpid.average_err=0;
		}*/
	
	  set_gimbal_speed(gimbal_y_speed,gimbal_p_speed);//(gimbal_y_angle,gimbal_p_angle);
}

void gimbal_angle_control(float yaw_angle,float pitch_angle)
{
 if(yaw_angle<=-120 || yaw_angle>=120 || pitch_angle>=60 || pitch_angle<=-60)
 {
 }
 else
 {
	yaw_angle = BASIC_YAW_ANGLE_CAN + yaw_angle*8191.0f/360.0f; 
	pitch_angle = BASIC_PITCH_ANGLE_CAN + pitch_angle*8191.0f/360.0f;
	set_gimbal_angle(yaw_angle,pitch_angle);
 }
}

	
// 函数: find_max()
// 描述: 找到计算得到的电机速度最大值
// 参数：无
// 输出：计算而得的电机最大值
// 内部函数，用户无需调用
int find_max()
{
  int temp=0;
  
  temp=abs(motor1.target_speed);
  if(abs(motor2.target_speed)>temp)
    temp=abs(motor2.target_speed);
  if(abs(motor3.target_speed)>temp)
    temp=abs(motor3.target_speed);
  if(abs(motor4.target_speed)>temp)
    temp=abs(motor4.target_speed);
  return temp;
}
//**********************************************************************************mhp111
//void trigger_angle_control(float trigger_angle)
//{	
//	if(trigger_angle >= bullet_angle*20)
//	{
//		trigger_angle-= bullet_angle*19;
//		motor5.total_angle-= bullet_angle*19;
//	}
//	set_trigger_angle(trigger_angle);
//}
int angle_judge_flag=0;
int angle_first_judge_flag=0;
void trigger_angle_control(float trigger_angle)
{	
	
	trigger_angle = trigger_angle/360*8191;
	if(angle_first_judge_flag==0)
	{
		motor5.apid.trigger_first_total_angle_storage=motor5.total_angle;
		angle_first_judge_flag++;
	}
	/*if(angle_judge_flag==0&&abs(motor5.apid.err>5000000))
	{
		motor5.apid.trigger_first_total_angle_storage=motor5.total_angle;
		angle_judge_flag++;
	}*/
	if(abs(motor5.apid.err<5000000))
		angle_judge_flag=0;
	if(trigger_angle>=(1774*100))
	{
	Kinematics.trigger.target_angle-=1774*99;
	motor5.total_angle-=1774*99;
	}
	set_trigger_angle(trigger_angle);
}


