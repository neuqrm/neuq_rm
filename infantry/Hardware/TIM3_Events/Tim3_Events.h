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
#include "mode.h"

void Robo_Move(void);							//�������˶�����
void Debug_Key(void);							//���԰���

#define pressed     0
#define unpressed   1

//�ٶȿ��ƺ����궨�壬ע�����г��õļ��������ɹ�ѡ��
#define Liner_X  Kinematics.target_velocities.linear_x
#define Liner_Y  Kinematics.target_velocities.linear_y
#define Angular_Z Kinematics.target_velocities.angular_z

/***** ��̨��������� *****/
#define Angular_Yaw Kinematics.yaw.target_angular        
                                              /*��Kinematics.yaw.target_angular)   ��λ���������
                                                 (pid_target_speed)       ���Գ���ʹ��																																																																		
                                              */
#define Angular_Pitch  Kinematics.pitch.target_angular
                                              /*(Kinematics.pitch.target_angular)
																							
																							*/
                                      
																			//ע�⣡���ٶȵ�Ӣ����angular velocity������Ϊ�˷���ȫ��ʹ��angular�������ٶȣ���ԭӢ���б�
#define Angle_Pitch Kinematics.pitch.target_angle
#define Angle_Yaw   Kinematics.yaw.target_angle

/***** �����ֿ����� *****/
#define Trigger_Speed Kinematics.trigger.target_angular
#define Trigger_Angle Kinematics.trigger.target_angle//mhp111
/***** Ħ���ֿ����� *****/
#define FRIC_Speed Kinematics.fric.target_angular
#define fric1_speed Kinematics.fric.target_angular
#define fric2_speed Kinematics.fric.target_angular//mhp222
/***** pid�����궨�� *****/
#define v_chassis_p  pid_t.chassis_pid.speed_loop.kp  
#define v_chassis_i  pid_t.chassis_pid.speed_loop.ki  
#define v_chassis_d  pid_t.chassis_pid.speed_loop.kd  

#define a_chassis_p  pid_t.chassis_pid.position_loop.kp  
#define a_chassis_i  pid_t.chassis_pid.position_loop.ki  
#define a_chassis_d  pid_t.chassis_pid.position_loop.kd  

#define v_trigger_p  pid_t.trigger_pid.speed_loop.kp  
#define v_trigger_i  pid_t.trigger_pid.speed_loop.ki  
#define v_trigger_d  pid_t.trigger_pid.speed_loop.kd  

#define a_trigger_p  pid_t.trigger_pid.position_loop.kp  
#define a_trigger_i  pid_t.trigger_pid.position_loop.ki  
#define a_trigger_d  pid_t.trigger_pid.position_loop.kd  

#define a_yaw_p      pid_t.yaw_pid.position_loop.kp   
#define a_yaw_i      pid_t.yaw_pid.position_loop.ki
#define a_yaw_d      pid_t.yaw_pid.position_loop.kd
#define v_yaw_p      pid_t.yaw_pid.speed_loop.kp
#define v_yaw_i      pid_t.yaw_pid.speed_loop.ki
#define v_yaw_d      pid_t.yaw_pid.speed_loop.kd
#define a_pitch_p    pid_t.pitch_pid.position_loop.kp
#define a_pitch_i    pid_t.pitch_pid.position_loop.ki
#define a_pitch_d    pid_t.pitch_pid.position_loop.kd
#define v_pitch_p    pid_t.pitch_pid.speed_loop.kp
#define v_pitch_i    pid_t.pitch_pid.speed_loop.ki
#define v_pitch_d   pid_t.pitch_pid.speed_loop.kd

#define v_fric1_p  pid_t.fric1_pid.speed_loop.kp  
#define v_fric1_i  pid_t.fric1_pid.speed_loop.ki  
#define v_fric1_d  pid_t.fric1_pid.speed_loop.kd  

#define v_fric2_p  pid_t.fric2_pid.speed_loop.kp  
#define v_fric2_i  pid_t.fric2_pid.speed_loop.ki  
#define v_fric2_d  pid_t.fric2_pid.speed_loop.kd  

/***** ����ʹ�� *****/
#define CHASSIS_BREAK_EN 0    // ����ʱ����ɲס��1ʹ�ܣ�0ʧ��
#define GIMBAL_POS_EN    1    // ������̨λ�û���1ʹ�ܣ�0ʧ��
#define TRIGGER_POS_EN   1    // ����������λ�û���1ʹ�ܣ�0ʧ��

#endif

