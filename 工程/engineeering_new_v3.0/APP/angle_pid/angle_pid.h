#ifndef _ANGLE_PID_H
#define _ANGLE_PID_H



#define set_trigger_motor_angle(motor5_angle) \
        do{ \
						motor5.apid.target_angle = motor5_angle; \
				}while(0)                                    \

//设置电机电机目标角度				
#define set_chassis_motor_angle(motor1_angle,motor2_angle,motor3_angle,motor4_angle) \
				do{  \
						motor1.apid.target_angle = motor1_angle; \
	          motor2.apid.target_angle = motor2_angle; \
	          motor3.apid.target_angle = motor3_angle; \
	          motor4.apid.target_angle = motor4_angle; \
        }while(0)                                    \
				
	


void APID_Init_All(void);			//电机机械角度PID参数初始化
void apid_chassic_realize(float kp,float ki,float kd);			//电机机械角度pid实现
void apid_handle_realize(float kp,float ki,float kd);

#endif
