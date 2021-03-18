#ifndef _KINEMATIC_H
#define _KINEMATIC_H

#define wheel_diameter  10.000000f			//轮子直径
#define half_width  25.000000f		//底盘半宽
#define half_length  35.000000f		//底盘半长

#define PI 			3.141593f
#define RPM2RAD 0.104720f										//转速转角速度		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.523599f										//转速转线速度		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//线速度转转度
#define M2006_REDUCTION_RATIO 36.000000f		//齿轮箱减速比
#define M3508_REDUCTION_RATIO 19.000000f		//齿轮箱减速比
#define GM6020_ENCODER_ANGLE  8192.0f


#define MAX_MOTOR_SPEED   15336				//电机最大转速，宏定义方便修改   范围0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    217.817f    //底盘最大平移速度，单位cm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f    //底盘最大旋转速度，单位rad/s    


#define set_trigger_speed(motor5_speed) \
	do{                                   \
		motor5.vpid.target_speed = motor5_speed; \
	    motor5.target_speed = motor5_speed;	     \
    }while(0)                                    \

#define set_gimbal_y_motor_speed(gimbal_y_speed) \
	do{                                    \
		gimbal_y.vpid.target_speed = gimbal_y_speed; \
	    gimbal_y.target_speed = gimbal_y_speed;		  \
    }while(0)                                     \

#define trigger_to_motor(trigger_angular)	\
	do{                               \
		motor5.target_speed =(int)(trigger_angular*M2006_REDUCTION_RATIO); \
	}while(0)                                                            \
												
#define set_gimbal_angle(yaw_angle,pitch_angle) \
	do{ \
		gimbal_y.apid.target_angle = yaw_angle; \
        gimbal_p.apid.target_angle = pitch_angle; \
	}while(0)                                   \

#define set_trigger_angle(trigger_angle) \
	do{ \
		motor5.apid.target_angle = trigger_angle+motor5.total_angle; \
	}while(0) //mhp111
#define set_fric_speed(fric1_speed,fric2_speed) \
	do{ \
		fric1.vpid.target_speed = fric1_speed; \
		fric2.vpid.target_speed = fric2_speed; \
	}while(0) //mhp111
	
typedef struct
{
	float linear_vel;			//线速度
	float rpm;						//转速圈每分钟
}Speed_t;

typedef struct
{
	Speed_t target_speed;			
	Speed_t actual_speed;						
}Wheel_t;

//底盘几何中心的线/角速度
typedef struct
{
	float linear_x;	//m/s
	float linear_y;
	float angular_z; //角速度rpm
}Velocities_t;

typedef struct
{
	float target_angular;
	float actual_angular;
  float target_angle;
	float actual_angle;
}Application_t;

typedef struct
{
	Wheel_t wheel1;
	Wheel_t wheel2;
	Wheel_t wheel3;
	Wheel_t wheel4;
	
	Velocities_t target_velocities;		//目标线速度
	Velocities_t actual_velocities; 	//实际线速度
	Application_t trigger;
	Application_t fric;
	Application_t pitch;
	Application_t yaw;
}Kinematics_t;




extern Kinematics_t Kinematics;
extern float max_base_linear_speed;
extern float max_base_rotational_speed;

extern int stop_flag_chassis;
extern int stop_flag_trigger;

void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Get_Base_Velocities(void);
void chassis_speed_control(float speed_x,float speed_y,float speed_r);		//将三个方向速度转换为电机转速
void trigger_control(float trigger_angular);
void trigger_angle_control(float trigger_angle);
void gimbal_speed_control(float gimbal_y_angle,float gimbal_p_angle);
void gimbal_angle_control(float yaw_angle,float pitch_angle);
void Get_Gimbal_Angle(void);
void fric_speed_control(float fric1_speed,float fric2_speed);//mhp111
#endif


