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
#include "string.h"
#include "stdio.h"
#include "gimbal.h"
int left_position;
int right_position;
//extern unsigned char left_ucRxData[100];
//extern unsigned char right_ucRxData[100];
//void Get_left_Base_position(unsigned char* left_ucFlag)//获取VL53L0数据
//{
//	unsigned char *p;
//	left_position = 0;
//	
//	if(*left_ucFlag==1)
//	{
//		p=(unsigned char*)strstr((char*)left_ucRxData,"d: ");
//		 
//		while((*p)!='m')
//		{
//			if(((*p)>='0')&&((*p)<='9')){
//				left_position=left_position*10+((*p)-'0');
//			}
//		}
//		//printf("距离:%dmm\r\n",position);
//		*left_ucFlag=0;
// }
//}

//void Get_right_Base_position(unsigned char* right_ucFlag)//获取VL53L0数据
//{
//	unsigned char *p;
//	right_position = 0;
//	
//	if(*right_ucFlag==1)
//	{
//		p=(unsigned char*)strstr((char*)right_ucRxData,"d: ");
//		 
//		while((*p)!='m')
//		{
//			if(((*p)>='0')&&((*p)<='9')){
//				right_position=right_position*10+((*p)-'0');
//			}
//		}
//		//printf("距离:%dmm\r\n",position);
//		*right_ucFlag=0;
// }
//}


Kinematics_t Kinematics;

//逆运动学公式
//把想要得到的底盘速度转换为轮子的线速度
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{	
	Kinematics.wheel1.target_speed.linear_vel = linear_x ;
	Kinematics.wheel2.target_speed.linear_vel = linear_x ;
	
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	
	motor1.target_speed =   (int)(Kinematics.wheel1.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor2.target_speed =   (int)(Kinematics.wheel2.target_speed.rpm * M3508_REDUCTION_RATIO);
}



//正运动学公式
//通过轮胎的实际转速计算底盘几何中心的三轴速度
void Get_Base_Velocities(void)
{
//	//根据电机转速测算轮子转速
//	Kinematics.wheel1.actual_speed.rpm = - motor1.actual_speed / M3508_REDUCTION_RATIO;
//	Kinematics.wheel2.actual_speed.rpm =   motor2.actual_speed / M3508_REDUCTION_RATIO;
//	Kinematics.wheel3.actual_speed.rpm =   motor3.actual_speed / M3508_REDUCTION_RATIO;
//	Kinematics.wheel4.actual_speed.rpm = - motor4.actual_speed / M3508_REDUCTION_RATIO;
//	//轮子转速转换为轮子线速度
//	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
//	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
//	Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
//	Kinematics.wheel4.actual_speed.linear_vel = Kinematics.wheel4.actual_speed.rpm * RPM2VEL;
//	//轮子线速度转换为底盘中心三轴的速度
//	Kinematics.actual_velocities.angular_z = ( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
//				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f*(half_width + half_length));
//	Kinematics.actual_velocities.linear_x  = (-Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
//				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
//	Kinematics.actual_velocities.linear_y  = ( Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
//				+ Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
	
	Kinematics.wheel1.actual_speed.rpm =   motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm =   motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
	Kinematics.actual_velocities.linear_x  = Kinematics.wheel2.actual_speed.linear_vel;
}

void Get_Gimbal_Angle()
{
  Kinematics.yaw.actual_angle = -(gimbal_y.actual_angle-BASIC_YAW_ANGLE_CAN)*(360.0f/GM6020_ENCODER_ANGLE);
  Kinematics.pitch.actual_angle = -(gimbal_p.actual_angle-BASIC_PITCH_ANGLE_CAN)*(360.0f/GM6020_ENCODER_ANGLE);
	if(Kinematics.yaw.actual_angle<0)
		Kinematics.yaw.actual_angle+=360;
	if(Kinematics.pitch.actual_angle<0)
		Kinematics.pitch.actual_angle+=360;
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
	if(stop_flag_chassis == 0 && speed_x == 0 && speed_y == 0 && speed_r == 0)
	{
		stop_flag_chassis = 1;			//停止   此标志为了避免多次进入
		stop_chassis_motor();			//停下来  并角度闭环
		motor1.target_speed= 0;
		motor2.target_speed= 0;

	}
	else if(speed_x != 0 || speed_y != 0 || speed_r != 0)
	{
		stop_flag_chassis = 0;
		//速度换算
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
 
		
		max=find_max();
		if(max>max_motor_speed)
		{
			motor1.target_speed=(int)(motor1.target_speed*max_motor_speed*1.0/max);
			motor2.target_speed=(int)(motor2.target_speed*max_motor_speed*1.0/max);
			motor3.target_speed=(int)(motor3.target_speed*max_motor_speed*1.0/max);
			motor4.target_speed=(int)(motor4.target_speed*max_motor_speed*1.0/max);
		}
	}
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
		motor5.target_speed=0;
	}
else if(trigger_angular!=0)
	{
		stop_flag_trigger = 0;
		
		trigger_to_motor(trigger_angular);
		
}
		set_trigger_speed(motor5.target_speed);		

	}


	
void gimbal_speed_control(float gimbal_y_speed,float gimbal_p_speed)    //
{
	
	 /*	if(gimbal_y_speed == 0)
	{
		stop_gimbal_motor();			//停下来  并角度闭环
	}
    else*/
    //gimbal_y_speed = KalmanFilter(gimbal_y_speed,1,200);
	  set_gimbal_speed(gimbal_y_speed,gimbal_p_speed);//(gimbal_y_angle,gimbal_p_angle);
}

void gimbal_angle_control(float yaw_angle,float pitch_angle)
{
	
  yaw_angle = BASIC_YAW_ANGLE_CAN - yaw_angle/360*8191; 
	pitch_angle = BASIC_PITCH_ANGLE_CAN - pitch_angle/360*8191;
		
	set_gimbal_angle(yaw_angle,pitch_angle);
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

