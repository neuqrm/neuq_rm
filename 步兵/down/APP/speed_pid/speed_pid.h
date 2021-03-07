#ifndef _SPEED_PID_H
#define _SPEED_PID_H
#include "motor.h"
#include "Kinematic.h"

/*3508������������Χ��     -16384 ~ 16384
                            ��Ӧ�������  -20A ~ 20A
	2006������������Χ��     -10000 ~ +10000
                            ��Ӧ�������  -10A ~ 10A	
	6020�����ѹ��Χ						-30000 ~ +30000
*/

#define CHASSIS_Integral_max         5000            //�����ֱ���
#define CHASSIS_IntegralSeparation   500             //���ַ���
#define CHASSIS_vPID_max             8000            //����޷�
#define TRIGGER_Integral_max         3000
#define TRIGGER_IntegralSeparation   50
#define TRIGGER_vPID_max             9000

#define GIMBAL_Integral_max         500
#define GIMBAL_IntegralSeparation   20
#define GIMBAL_vPID_max             20000

//�궨�壬���Ŀ���ٶȸ���ֵ
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
					/*�����ٶȻ�*/                        \
					pid_t.chassis_pid.speed_loop.kp = 2; \
					pid_t.chassis_pid.speed_loop.ki = 0.05; \
					pid_t.chassis_pid.speed_loop.kd = 0; \
					                                     \
					/*����λ�û�*/                        \
					pid_t.chassis_pid.position_loop.kp = 0.2; \
					pid_t.chassis_pid.position_loop.ki = 0.05; \
					pid_t.chassis_pid.position_loop.kd = 0; \
                                               \
					/*�������ٶȻ�*/                      \
					pid_t.trigger_pid.speed_loop.kp = 2.5; \
					pid_t.trigger_pid.speed_loop.ki = 0.05; \
					pid_t.trigger_pid.speed_loop.kd = 0; \
                                               \
					/*��̨yawλ�û�*/                    \
					pid_t.yaw_pid.position_loop.kp = 0; \
					pid_t.yaw_pid.position_loop.ki = 0; \
					pid_t.yaw_pid.position_loop.kd = 0; \
					                                    \
					/*��̨yaw�ٶȻ�*/                    \
				  pid_t.yaw_pid.speed_loop.kp = 0; \
				  pid_t.yaw_pid.speed_loop.ki = 0; \
				  pid_t.yaw_pid.speed_loop.kd = 0; \
					                                 \
					/*��̨pitchλ�û�*/                    \
					pid_t.pitch_pid.position_loop.kp = 0; \
					pid_t.pitch_pid.position_loop.ki = 0; \
					pid_t.pitch_pid.position_loop.kd = 0; \
					                                      \
					/*��̨pitch�ٶȻ�*/                    \
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

void VPID_Init_All(void);			//���ת��PID������ʼ��

void vpid_chassis_realize(float kp,float ki,float kd);				//���ת��PIDʵ��
void vpid_trigger_realize(float kp,float ki,float kd);      //����
void vpid_gimbal_realize(float kp_y,float ki_y,float kd_y,float kp_p,float ki_p,float kd_p); //��̨�ٶȻ�

int abs(int input);				//�����ֵ����
int pid_auto(void);
int pid_pc(void);
#endif
