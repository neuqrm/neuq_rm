/**
  ******************************************************************************
  * @file    Project/APP/kinematic.c 
  * @author  Siyuan Qiao&Junyu Luo 
  * @version V1.0.0
  * @date    2.2020
  * @brief   ���������˶�ѧ����
  *          ���ٶȵ�λ�� cm/s
  *          ���ٶȵ�λ�� rad/s
	*          ת�ٵ�λ��   rpm
  ******************************************************************************
  * @attention
  ******************************************************************************
  */


#include "kinematic.h"
#include "motor.h"
#include "speed_pid.h"
#include "algorithm.h"
#include "gimbal.h"
#include "encoder.h"
#include "power_limitation.h"
Kinematics_t Kinematics;

//���˶�ѧ��ʽ
//����Ҫ�õ��ĵ����ٶ�ת��Ϊ���ӵ����ٶ�
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.wheel1.target_speed.linear_vel = -linear_x - linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel2.target_speed.linear_vel = -linear_x + linear_y - angular_z*(half_width+half_length);
	Kinematics.wheel3.target_speed.linear_vel = -linear_x + linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel4.target_speed.linear_vel = -linear_x -  linear_y - angular_z*(half_width+half_length);
	//���ٶ� cm/s  תת��  RPM 
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel4.target_speed.rpm = Kinematics.wheel4.target_speed.linear_vel * VEL2RPM;
	
	motor1.target_speed = - (int)(Kinematics.wheel1.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor2.target_speed =   (int)(Kinematics.wheel2.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor3.target_speed = - (int)(Kinematics.wheel3.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor4.target_speed =  (int)(Kinematics.wheel4.target_speed.rpm * M3508_REDUCTION_RATIO);
	
}



//���˶�ѧ��ʽ
//ͨ����̥��ʵ��ת�ټ�����̼������ĵ������ٶ�
void Get_Base_Velocities(void)
{
	//���ݵ��ת�ٲ�������ת��
	Kinematics.wheel1.actual_speed.rpm = - motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm =   motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel3.actual_speed.rpm =   motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel4.actual_speed.rpm = - motor4.actual_speed / M3508_REDUCTION_RATIO;
	//����ת��ת��Ϊ�������ٶ�
	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel4.actual_speed.linear_vel = Kinematics.wheel4.actual_speed.rpm * RPM2VEL;
	//�������ٶ�ת��Ϊ��������������ٶ�
	Kinematics.actual_velocities.angular_z = ( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f*(half_width + half_length));
	Kinematics.actual_velocities.linear_x  = (-Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
	Kinematics.actual_velocities.linear_y  = ( Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				+ Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
}

void Get_Gimbal_Angle()   
{
	if(gimbal_y.actual_angle>=0&&gimbal_y.actual_angle<=BASIC_YAW_ANGLE_CAN)
		gimbal_y.actual_angle=gimbal_y.actual_angle+8191;
	
	if(gimbal_y.actual_angle<BASIC_YAW_ANGLE_CAN+4096)
	Kinematics.yaw.actual_angle=(gimbal_y.actual_angle-BASIC_YAW_ANGLE_CAN)/GM6020_ENCODER_ANGLE*2*PI/2;
	else if(gimbal_y.actual_angle>=BASIC_YAW_ANGLE_CAN+4096)  
		Kinematics.yaw.actual_angle=2*PI-(2*PI-((gimbal_y.actual_angle-BASIC_YAW_ANGLE_CAN)/GM6020_ENCODER_ANGLE*2*PI))*2;
  
  Kinematics.pitch.actual_angle = (gimbal_p.actual_angle-BASIC_PITCH_ANGLE_CAN)*(360.0f/GM6020_ENCODER_ANGLE);
}
//�õ�Ħ���ֵ������ٶ� 20ms
//ÿ�������� 266*4/(0.05*3.14)=6777
int last_fire1_speed=0,last_fire2_speed=0;
void Get_frie_speed()
{
	fire2.actual_speed=get_left_encoder_speed();  //m/s*50/6777*50/6777
	fire1.actual_speed=get_right_encoder_speed();
	//��ȥ�쳣ֵ
	if(fire2.actual_speed-last_fire2_speed>=400)
	{
		fire2.actual_speed=last_fire2_speed;
	}
	else last_fire2_speed=fire2.actual_speed;
	
	if(fire1.actual_speed-last_fire1_speed>=400)
	{
		fire1.actual_speed=last_fire1_speed;
	}
	else last_fire1_speed=fire1.actual_speed;
}

// ����: speed_control()
// ����: ��pid�ٶ����ת��Ϊ����ٶȣ����մ��ݸ��ٶ�pid
// ����������������ٶ�
// �����4������ٶ�
// ע�����1��4��Ĭ����ת����ͳ���ʵ���������෴����Ҫȡ��
int find_max(void);
int stop_flag_chassis=0;
void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
		stop_flag_chassis = 0;
		//�ٶȻ���
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
 
		power_limitation_jugement();
		max=find_max();
		if(max>max_motor_speed)
		{
			motor1.target_speed=(int)(motor1.target_speed*max_motor_speed*1.0/max);
			motor2.target_speed=(int)(motor2.target_speed*max_motor_speed*1.0/max);
			motor3.target_speed=(int)(motor3.target_speed*max_motor_speed*1.0/max);
			motor4.target_speed=(int)(motor4.target_speed*max_motor_speed*1.0/max);
		}
	set_chassis_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
}	

int stop_flag_trigger=0;

void trigger_speed_angle(float trigger_angular)
{
if(stop_flag_trigger == 0 && trigger_angular==0)
	{
		stop_flag_trigger = 1;			//ֹͣ   �˱�־Ϊ�˱����ν���
		stop_trigger_motor();			//ͣ����  ���Ƕȱջ�
	}
else if(trigger_angular!=0)
	{
		stop_flag_trigger = 0;
		
		trigger_to_motor(trigger_angular);
		
		set_trigger_speed(motor5.target_speed);		
}
	}

int flag_1=1;
int flag_2=1;
	
void gimbal_speed_control(float gimbal_y_speed,float gimbal_p_speed)    //
{
	
	 /*	if(gimbal_y_speed == 0)
	{
		stop_gimbal_motor();			//ͣ����  ���Ƕȱջ�
	}
    else*/
    //gimbal_y_speed = KalmanFilter(gimbal_y_speed,1,200);
	 /*if(Kinematics.yaw.actual_angle<=87)
			flag_1=1;
		if(Kinematics.yaw.actual_angle>280&&gimbal_y.vpid.actual_speed>0)
			flag_1=0;
		if(flag_1==0)
		{
		Kinematics.yaw.target_angular=-40;
		flag_1=2;
		}
	   if(flag_1==1)
		{
		
		Kinematics.yaw.target_angular=40;
		//gimbal_p.vpid.PID_OUT=0;
		//gimbal_p.vpid.average_err=0;
		flag_1=2;
		}*/
	  set_gimbal_speed(gimbal_y_speed,gimbal_p_speed);//(gimbal_y_angle,gimbal_p_angle);
}
void gimbal_angle_control(float yaw_angle,float pitch_angle)
{
	/*if(Kinematics.yaw.actual_angle>=5.2) flag_2=1;
	if(flag_2==1)
	{yaw_angle=1.75;}
	if(Kinematics.yaw.actual_angle<=3.6)   flag_2=2;
	if(flag_2==2)
	{yaw_angle=2.65;}*/
	
	//��λ 2360  2730  7821  5090
	/*if(yaw_angle>=50) yaw_angle=50;
    else  if(yaw_angle<=-50) yaw_angle=-50;
  	
	if(pitch_angle>=30) pitch_angle=30;
    else  if(pitch_angle<=-30) pitch_angle=-30;*/
	
	//yaw_angle =BASIC_YAW_ANGLE_CAN+yaw_angle*8191.0f/360.0f; 
	
	
	if(yaw_angle>=1.382&&yaw_angle<=PI) yaw_angle=1.382f; //80��
	else if(yaw_angle>=PI&&yaw_angle<=4.887)  yaw_angle=4.887f;  //280��

	/*if(yaw_angle>=0&&yaw_angle<=1.571)	 yaw_angle=yaw_angle*2.0f;
	if(yaw_angle>=4.712&&yaw_angle<=PI2)  yaw_angle=PI2-2*(PI2-yaw_angle);*/
	yaw_angle=yaw_angle*1.0f;
	pitch_angle = BASIC_PITCH_ANGLE_CAN + pitch_angle*8191.0f/360.0f;
	set_gimbal_angle(yaw_angle,pitch_angle);
}

	
// ����: find_max()
// ����: �ҵ�����õ��ĵ���ٶ����ֵ
// ��������
// �����������õĵ�����ֵ
// �ڲ��������û��������
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
int angle_judge_flag=0;
void trigger_angle_control(int trigger_angle)
{	
	trigger_angle = trigger_angle/360*8191;

	if(motor5.round_cnt>=40)
	{
		Kinematics.trigger.target_angle-=motor5.round_cnt*8192/19;
		if(Kinematics.trigger.target_angle<=0) Kinematics.trigger.target_angle=1;
		motor5.total_angle -=motor5.round_cnt*8192 ;
	    motor5.start_angle=motor5.total_angle;
		motor5.round_cnt=0;
	}
	set_trigger_angle(trigger_angle);
}
