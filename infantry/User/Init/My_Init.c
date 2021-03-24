
/*****     此文件专门用来放各个初始化函数    *****/

#include "My_Init.h"
#include "stm32f4xx_rcc.h"
#include "delay.h"
#include "key.h"
#include "LED.h"
#include "myflash.h"
#include "timer.h"
#include "can.h"
#include "power.h"
#include "fric.h"
#include "motor.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "DJi_remote.h"
#include "remote_code.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "gimbal.h"
#include "bsp_imu_usart.h"
#include "steering_engine.h"

RCC_ClocksTypeDef get_rcc_clock;		

void All_Init()
{
	motor5.round_cnt=0;
	Stm32_Clock_Init(360,12,2,8);				
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(180);									
	power_init();										 
	power_open_all();									
	Dji_Remote_Init();									
	led_init();											
	key_init();											
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				
	Debug_USART_Config();           
	JSON_USART_Config();           	
	bsp_imu_usart_init();
	motor_init();		
	fric_PWM_configuration();      
	TIM1_GIMBAL_Init();
	TIM3_Int_Init(10-1,9000-1);		 
	VPID_Init_All();								
	APID_Init_All();								
	stop_chassis_motor();									
	TIM_Steering_Engine_PWM_Init();
	RCC_GetClocksFreq(&get_rcc_clock); 			
	fric_record_control();//mhp111
}
