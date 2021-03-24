/**
  ******************************************************************************
  * @file    Project/HARDWARE/motor.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................

*/  
#include "motor.h"
#include "can.h"
#include "delay.h"
#include "angle_pid.h"
#include "kinematic.h"
#include "algorithm.h"

MOTOR_t motor1,motor2,motor3,motor4,motor5,motor6,gimbal_y,gimbal_p,fric1,fric2;
LOOPBACK loopback;

int Q=0.01;
int R=0.01;

int max_motor_speed=MAX_MOTOR_SPEED;		
float max_base_linear_speed=MAX_BASE_LINEAR_SPEED; 
float max_base_rotational_speed=MAX_BASE_ROTATIONAL_SPEED;  
int callback_flag=1;
	
void record_motor_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = 0.5*(motor->last_angle + angle);
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;
	motor->last_angle = angle;
	motor->actual_current = current;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;
	}
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->total_angle = motor->round_cnt * 8191 + motor->actual_angle;
}

void record_trigger_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;
	motor->actual_current = current;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	
	}
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->total_angle = motor->round_cnt * 8191 + motor->actual_angle;
}

void record_gimbal_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	static int temp_speed1,temp_speed2,temp_speed3,temp_angle1,temp_angle2,temp_angle3;
	static int maxangle,minangle,maxspeed,minspeed;
	motor->last_angle = motor->actual_angle;
	motor->actual_current = current;
	switch(callback_flag)
	{
		case(1):
		{
			temp_angle1=angle;
			temp_speed1=speed;
			callback_flag=2;
		}
		break;
		case(2):
		{
			temp_angle2=angle;
			temp_speed2=speed;
			callback_flag=3;
		}
		break;
		case(3):
		{
			temp_angle3=angle;
			temp_speed3=speed;
			maxspeed=(temp_speed1>temp_speed2?temp_speed1:temp_speed2);
			maxspeed=(maxspeed>temp_speed3?maxspeed:temp_speed3);
			minspeed=(temp_speed1<temp_speed2?temp_speed1:temp_speed2);
			minspeed=(maxspeed<temp_speed3?minspeed:temp_speed3);
			maxangle=(temp_angle1>temp_angle2?temp_angle1:temp_angle2);
			maxangle=(maxangle>temp_angle3?maxspeed:temp_angle3);
			minangle=(temp_angle1<temp_angle2?temp_angle1:temp_angle2);
			minangle=(maxangle<temp_angle3?minangle:temp_angle3);
			motor->actual_angle =(temp_angle1+temp_angle2+temp_angle3)-maxangle-minangle;
			motor->actual_speed =(temp_speed1+temp_speed2+temp_speed3)-maxspeed-minspeed;
			callback_flag=1;
		}
		break;
		default:
		break;
	}
}
	

/**
  * @breif 电机参数初始化
  */
void motor_init()
{

	motor1.start_angle = 0;
	motor1.actual_angle = 0;
	motor1.actual_speed = 0;
	motor1.start_angle_flag = 0;
	motor1.actual_current = 0;
	motor1.target_current = 0;
	motor2.start_angle = 0;
	motor2.actual_angle = 0;
	motor2.start_angle_flag = 0;
	motor2.actual_speed = 0;
	motor2.actual_current = 0;
	motor2.target_current = 0;
	motor3.start_angle = 0;
	motor3.actual_angle = 0;
	motor3.start_angle_flag = 0;
	motor3.actual_speed = 0;
	motor3.actual_current = 0;
	motor3.target_current = 0;
	motor4.start_angle = 0;
	motor4.actual_angle = 0;
	motor4.start_angle_flag = 0;
	motor4.actual_speed = 0;
	motor4.actual_current = 0;
	motor4.target_current = 0;
	gimbal_y.start_angle = 0;
	gimbal_y.actual_angle = 0;
	gimbal_y.start_angle_flag = 0;
	gimbal_y.actual_speed = 0;
	gimbal_y.actual_current = 0;
	gimbal_y.target_current = 0;
	gimbal_p.start_angle = 0;
	gimbal_p.actual_angle = 0;
	gimbal_p.start_angle_flag = 0;
	gimbal_p.actual_speed = 0;
	gimbal_p.actual_current = 0;
	gimbal_p.target_current = 0;
}

void set_chassis_current()
{
	u8 current_msg[8];
	motor1.target_current = motor1.vpid.PID_OUT;
	motor2.target_current = motor2.vpid.PID_OUT;
	motor3.target_current = motor3.vpid.PID_OUT;
	motor4.target_current = motor4.vpid.PID_OUT;
	current_msg[0] = motor1.target_current >> 8;			
	current_msg[1] = motor1.target_current & 0xff;		
	current_msg[2] = motor2.target_current >> 8;			
	current_msg[3] = motor2.target_current & 0xff;		
	current_msg[4] = motor3.target_current >> 8;		
	current_msg[5] = motor3.target_current & 0xff;		
	current_msg[6] = motor4.target_current >> 8;			
	current_msg[7] = motor4.target_current & 0xff;		
	CAN1_Send_CHASSIS_Msg(current_msg);
}

void set_trigger_current()
{
	u8 current_msg[8];
	motor5.target_current = motor5.vpid.PID_OUT;//
	current_msg[0] =motor5.target_current >> 8;			
	current_msg[1] = motor5.target_current & 0xff;		
	CAN1_Send_Trigger_Msg(current_msg);
}


void set_gimbal_current()
{
	u8 current_msg[8];
	gimbal_y.target_current = gimbal_y.vpid.PID_OUT; 
	gimbal_p.target_current = gimbal_p.vpid.PID_OUT; 
	current_msg[0] =gimbal_y.target_current >> 8;			
	current_msg[1] =gimbal_y.target_current & 0xff;		
	current_msg[2] =gimbal_p.target_current >> 8;
	current_msg[3] =gimbal_p.target_current & 0xff;		
	CAN1_Send_GIMBAL_Msg(current_msg);
}
void stop_chassis_motor()
{
	motor1.stop_angle = motor1.total_angle;
	motor2.stop_angle = motor2.total_angle;
	motor3.stop_angle = motor3.total_angle;
	motor4.stop_angle = motor4.total_angle;
	set_chassis_motor_angle(motor1.stop_angle,motor2.stop_angle,motor3.stop_angle,motor4.stop_angle);
}

void stop_trigger_motor()
{
	motor5.stop_angle = motor5.actual_angle;
	set_trigger_motor_angle(motor5.stop_angle);
}
