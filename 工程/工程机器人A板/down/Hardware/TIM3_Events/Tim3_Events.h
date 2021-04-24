#ifndef _TIM3_EVENTS_H
#define _TIM3_EVENTS_H

#include <string.h>
#include <stdio.h>
#include "key.h"
#include "led.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "fric.h"
#include "motor.h"
#include "kinematic.h"
#include "remote_code.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "stm32f4xx_tim.h"
#include "gimbal.h"
#include "json.h"
#include "angle_pid.h"

void Robo_Move(void);							//�������˶�����
void Debug_Key(void);							//���԰���

#define pressed     0
#define unpressed   1

//�ٶȿ��ƺ����궨�壬ע�����г��õļ��������ɹ�ѡ��
#define Liner_X  Kinematics.target_velocities.linear_x
#define Liner_Y  Kinematics.target_velocities.linear_y
#define Angular_Z Kinematics.target_velocities.angular_z

#define Handle_L (motor5.start_angle + 5*8192)        
                                              /*��Kinematics.yaw.target_angular)   ��λ���������
                                                 (pid_target_speed)       ���Գ���ʹ��																																																																		
                                              */
#define Handle_R (motor6.start_angle - 5*8192)
                                              /*(Kinematics.pitch.target_angular)
																							
																							*/
                                      
																			//ע�⣡���ٶȵ�Ӣ����angular velocity������Ϊ�˷���ȫ��ʹ��angular������ٶȣ���ԭӢ���б�

#define Handle_speed Kinematics.handle_L.target_angular

#define Angle_Pitch gimbal_p.apid.target_angle
#define Angle_Yaw   gimbal_y.apid.target_angle

/***********pid�����궨��************/
#define v_chassic_p  pid_t.chassic_pid.speed_loop.kp  
#define v_chassic_i  pid_t.chassic_pid.speed_loop.ki  
#define v_chassic_d  pid_t.chassic_pid.speed_loop.kd  

#define v_trigger_p  pid_t.trigger_pid.speed_loop.kp  
#define v_trigger_i  pid_t.trigger_pid.speed_loop.ki  
#define v_trigger_d  pid_t.trigger_pid.speed_loop.kd  

#define v_handle_p pid_t.handle.speed_loop.kp
#define v_handle_i pid_t.handle.speed_loop.ki
#define v_handle_d pid_t.handle.speed_loop.kd
#define a_handle_p pid_t.handle.position_loop.kp
#define a_handle_i pid_t.handle.position_loop.ki
#define a_handle_d pid_t.handle.position_loop.kd



#endif


