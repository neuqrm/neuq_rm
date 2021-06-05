
/*****     此文件专门用来放各个初始化函数    *****/

#include "My_Init.h"
#include "delay.h"
#include "key.h"
#include "LED.h"
#include "timer.h"

#include "can.h"
#include "power.h"

#include "motor.h"
#include "speed_pid.h"
#include "angle_pid.h"

#include "remote_code.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "bsp_dbus.h"
#include "bsp_imu_usart.h"
#include "steering_engine.h"
#include "stepper_motor.h"
#include "limit_switch.h"

RCC_ClocksTypeDef get_rcc_clock;		//系统时钟结构体

// 函数: All_Init()
// 描述: 机器人所有参数初始化
// 参数：无
// 输出：无

void All_Init()
{
	Stm32_Clock_Init(360,12,2,8);				//设置时钟,180Mhz = 12M / 25 * 350 / 2
	pid_init();                     //初始化pid各项参数的值

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	delay_init(180);									//延时初始化
	
	power_init();										  //电机电源初始化，默认为关
	power_close_all();									//开启所有电机电源
	Dji_Remote_Init();									//大疆遥控器初始化

	led_init();											//led初始化
	key_init();											//按键初始化

	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				//can初始化（电机初始化） 45M/(4+4+1)/5

	Debug_USART_Config();           //通信串口初始化（USART2）
	

	Stepper_Motor_VCC_GPIO_Init(); 

	motor_init();		//电机参数初始化

	VPID_Init_All();								//速度pid参数初始化
	APID_Init_All();								//角度pid参数初始化
	//stop_chassis_motor();									//让电机保持当前角度（锁死电机）
	step_motor_init();
	
	TIM4_NVIC_Init();
	TIM_Steering_Engine_PWM_Init();
	
	TIM_Stepper_Motor_PWM_Init();
	Stepper_Motor_EN_GPIO_Init();
	Stepper_Motor_DIR_GPIO_Init();
	
	//limit_switch_NVIC_Configuration();
	limit_init_test();
	RCC_GetClocksFreq(&get_rcc_clock); 			//查看系统时钟(watch中)
	TIM3_Int_Init(10-1,9000-1);		  //定时器时钟90M，9000，所以90M/9000=10Khz的计数频率，计数10次为1ms  即1khz
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
		
}
		


void software_init()   
{
	motor_init();		//电机参数初始化
	pid_init();                     //初始化pid各项参数的值
	remote_ch_init();
	VPID_Init_All();								//速度pid参数初始化
	APID_Init_All();								//角度pid参数初始化
}

void hardware_init()
{
	Stm32_Clock_Init(360,12,2,8);				//设置时钟,180Mhz = 12M / 25 * 350 / 2
	RCC_GetClocksFreq(&get_rcc_clock); 			//查看系统时钟(watch中)
	delay_init(180);									//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	
	Stepper_Motor_VCC_GPIO_Init();
	TIM4_NVIC_Init();
	power_init();										  //电机电源初始化，默认为关
	led_init();											//led初始化
	key_init();											//按键初始化
	Stepper_Motor_EN_GPIO_Init();
	Stepper_Motor_DIR_GPIO_Init();
	step_motor_init();

	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				//can初始化（电机初始化） 45M/(4+4+1)/5
	Dji_Remote_Init();									//大疆遥控器初始化
	Debug_USART_Config();           //通信串口初始化（USART2）

	TIM_Steering_Engine_PWM_Init();
	TIM_Stepper_Motor_PWM_Init();
	limit_switch_NVIC_Configuration();
	TIM3_Int_Init(10-1,9000-1);		  //定时器时钟90M，9000，所以90M/9000=10Khz的计数频率，计数10次为1ms  即1khz
	power_close_all();
	stop_chassis_motor();									//让电机保持当前角度（锁死电机）
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
}
