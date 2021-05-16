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
#include "remote.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "stm32f4xx_tim.h"
#include "gimbal.h"
#include "json.h"
#include "angle_pid.h"
#include "mode.h"

void Robo_Move(void);							//机器人运动控制
void Debug_Key(void);							//调试按键

#define pressed     0
#define unpressed   1

//速度控制函数宏定义，注释里有常用的几个变量可供选择
#define Liner_X  Kinematics.target_velocities.linear_x
#define Liner_Y  Kinematics.target_velocities.linear_y
#define Angular_Z Kinematics.target_velocities.angular_z

/***** 云台输入控制量 *****/
#define Angular_Yaw Kinematics.yaw.target_angular        
                                              /*（Kinematics.yaw.target_angular)   上位机传入参数
                                                 (pid_target_speed)       测试程序使用																																																																		
                                              */
#define Angular_Pitch  Kinematics.pitch.target_angular
                                              /*(Kinematics.pitch.target_angular)
																							
																							*/
                                      
																			//注意！角速度的英文是angular velocity，这里为了方便全部使用angular代表角速度，与原英文有别
#define Angle_Pitch Kinematics.pitch.target_angle
#define Angle_Yaw   Kinematics.yaw.target_angle

/***** 拨弹轮控制量 *****/
#define Trigger_Speed Kinematics.trigger.target_angular
#define Trigger_Angle Kinematics.trigger.target_angle
/***** 摩擦轮控制量 *****/
#define FRIC1_Speed Kinematics.fric1.target_angular
#define FRIC2_Speed Kinematics.fric2.target_angular
/***** pid参数宏定义 *****/
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

#define v_fire_p  pid_t.fire_pid.speed_loop.kp  
#define v_fire_i  pid_t.fire_pid.speed_loop.ki  
#define v_fire_d  pid_t.fire_pid.speed_loop.kd  

/***** 功能使能 *****/
#define CHASSIS_BREAK_EN 0    // 启动时底盘刹住 1使能，0失能
#define GIMBAL_POS_EN    1    // 启动云台位置环，1使能，0失能
#define TRIGGER_POS_EN   1// 启动拨弹轮位置环，1使能，0失能
#define FRIC_CLOSE_LOOP_EN 1 //启动摩擦轮闭环 ，1 使能， 0失能

#endif


