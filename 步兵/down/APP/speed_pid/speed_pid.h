#ifndef _SPEED_PID_H
#define _SPEED_PID_H
#include "motor.h"
#include "Kinematic.h"

/*3508电机电调电流范围：     -16384 ~ 16384
                            对应电调电流  -20A ~ 20A
	2006电机电调电流范围：     -10000 ~ +10000
                            对应电调电流  -10A ~ 10A	
	6020电机电压范围						-30000 ~ +30000
*/

#define CHASSIS_Integral_max         5000            //抗积分饱和
#define CHASSIS_IntegralSeparation   500             //积分分离
#define CHASSIS_vPID_max             8000            //输出限幅
#define TRIGGER_Integral_max         3000
#define TRIGGER_IntegralSeparation   50
#define TRIGGER_vPID_max             9000

#define GIMBAL_Integral_max         500
#define GIMBAL_IntegralSeparation   20
#define GIMBAL_vPID_max             20000

//宏定义，电机目标速度复赋值
#define set_chassis_speed(motor1_speed,motor2_speed,motor3_speed,motor4_speed) \
        do{                                                                    \
					motor1.vpid.target_speed = motor1_speed;		                         \
	        motor2.vpid.target_speed = motor2_speed;                             \
	        motor3.vpid.target_speed = motor3_speed;                             \
	        motor4.vpid.target_speed = motor4_speed;                             \
	        motor1.target_speed = motor1_speed;		                               \
	        motor2.target_speed = motor2_speed;                                  \
	        motor3.target_speed = motor3_speed;                                  \
	        motor4.target_speed = motor4_speed;                                  \
				}while(0)                                                              \


#define set_gimbal_speed(yaw_speed,pitch_speed) \
        do{                                     \
           gimbal_y.vpid.target_speed = yaw_speed; \
					 gimbal_y.target_speed = yaw_speed;       \
         	 gimbal_p.vpid.target_speed = pitch_speed; \
					 gimbal_p.target_speed = pitch_speed;      \
        }while(0)					                          \

#define pid_init() \
				do{ \
					/*底盘速度环*/                        \
					pid_t.chassis_pid.speed_loop.kp = 2; \
					pid_t.chassis_pid.speed_loop.ki = 0.05; \
					pid_t.chassis_pid.speed_loop.kd = 0; \
					                                     \
					/*底盘位置环*/                        \
					pid_t.chassis_pid.position_loop.kp = 0.2; \
					pid_t.chassis_pid.position_loop.ki = 0.05; \
					pid_t.chassis_pid.position_loop.kd = 0; \
                                               \
					/*拨弹轮速度环*/                      \
					pid_t.trigger_pid.speed_loop.kp = 2.5; \
					pid_t.trigger_pid.speed_loop.ki = 0.05; \
					pid_t.trigger_pid.speed_loop.kd = 0; \
                                               \
					/*云台yaw位置环*/                    \
					pid_t.yaw_pid.position_loop.kp = 0; \
					pid_t.yaw_pid.position_loop.ki = 0; \
					pid_t.yaw_pid.position_loop.kd = 0; \
					                                    \
					/*云台yaw速度环*/                    \
				  pid_t.yaw_pid.speed_loop.kp = 0; \
				  pid_t.yaw_pid.speed_loop.ki = 0; \
				  pid_t.yaw_pid.speed_loop.kd = 0; \
					                                 \
					/*云台pitch位置环*/                    \
					pid_t.pitch_pid.position_loop.kp = 0; \
					pid_t.pitch_pid.position_loop.ki = 0; \
					pid_t.pitch_pid.position_loop.kd = 0; \
					                                      \
					/*云台pitch速度环*/                    \
				  pid_t.pitch_pid.speed_loop.kp = 0; \
				  pid_t.pitch_pid.speed_loop.ki = 0; \
				  pid_t.pitch_pid.speed_loop.kd = 0; \
				}while(0)                            \
				
typedef enum switch_flag_t
{
	
	CHASSIS = 1,
	TRIGGER = 2,
	GIMBAL  = 3,
	NUL=0,
}switch_flag_t;

typedef struct
{
	float kp;			
	float ki;	
  float kd;	
}Parameter_t;

typedef struct
{
  Parameter_t position_loop;
	Parameter_t speed_loop;
}PID_Loop_t;
				
typedef struct
{
  PID_Loop_t chassis_pid;
	PID_Loop_t trigger_pid;
	PID_Loop_t yaw_pid;
	PID_Loop_t pitch_pid;
}PID_t;

extern PID_t pid_t;		
extern int pid_target_speed;
extern int pid_target_angle;
extern switch_flag_t switch_flag;

void VPID_Init_All(void);			//电机转速PID参数初始化

void vpid_chassis_realize(float kp,float ki,float kd);				//电机转速PID实现
void vpid_trigger_realize(float kp,float ki,float kd);      //拨弹
void vpid_gimbal_realize(float kp_y,float ki_y,float kd_y,float kp_p,float ki_p,float kd_p); //云台速度环

int abs(int input);				//求绝对值函数
int pid_auto(void);
int pid_pc(void);
#endif
