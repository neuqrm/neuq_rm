/**
  ******************************************************************************
  * @file    Project/USER/flag.c 
  * @author  neuq robomaster
  * @version V1.0.0
  * @date    5.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/  
#include "flag.h"
/*----------- 电磁阀标识符-----------*/
uint16_t Solenoid_valve_flag = 0;     //伸出机构电磁阀
uint16_t Solenoid_handle_flag = 0;    //抓手电磁阀

/*----------- 运动学控制标识符 ------------*/
uint16_t stop_flag_1=0;
uint16_t stop_flag_4=0;
uint16_t test_flag=0;
uint16_t handle_test_flag=0;
uint16_t stop_flag_2=0;

/*---------- 电机脱力标识符 ---------------*/
uint16_t off_line_flag=0;


/*---------- 按键标识符 -----------*/
uint16_t lever_flag=0;   //判断拨杆是否上下一个来回
uint16_t lever_start=0;
uint16_t key_flag=0;     //判断按键是否长按

