#ifndef _TIM3_EVENTS_H
#define _TIM3_EVENTS_H

#include <string.h>
#include <stdio.h>
#include "key.h"
#include "led.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"

#include "motor.h"
#include "kinematic.h"
#include "remote_code.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "stm32f4xx_tim.h"
#include "steering_engine.h"
#include "stepper_motor.h"
#include "limit_switch.h"
#include "flag.h"
#include "power.h"

#include "angle_pid.h"

void Robo_Move(void);							//机器人运动控制
void Debug_Key(void);							//调试按键

#define pressed     0
#define unpressed   1

/***********pid参数宏定义************/
#define v_chassic_p  pid_t.chassic_pid.speed_loop.kp  
#define v_chassic_i  pid_t.chassic_pid.speed_loop.ki  
#define v_chassic_d  pid_t.chassic_pid.speed_loop.kd  

#define v_handle_p  pid_t.handle.speed_loop.kp 
#define v_handle_i  pid_t.handle.speed_loop.ki
#define v_handle_d  pid_t.handle.speed_loop.kd

#define a_handle_p  pid_t.handle.position_loop.kp 
#define a_handle_i  pid_t.handle.position_loop.ki
#define a_handle_d  pid_t.handle.position_loop.kd

#define HANDLE_POS_EN 1

#endif


