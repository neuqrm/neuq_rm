
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
#include "bsp_dbus.h"
#include "remote.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "gimbal.h"
#include "bsp_imu_usart.h"
#include "encoder.h"
#include "bsp_referee_usart.h"
#include "referee.h"

RCC_ClocksTypeDef get_rcc_clock;		//系统时钟结构体

// 函数: All_Init()
// 描述: 机器人所有参数初始化
// 参数：无
// 输出：无
void All_Init()
{
	Stm32_Clock_Init(360,12,2,8);				//设置时钟,180Mhz = 12M / 25 * 350 / 2
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	delay_init(180);									//延时初始化
	delay_init(180);									//延时初始化
	power_init();										  //电机电源初始化，默认为关
	power_open_all();									//开启所有电机电源
	
	Dji_Remote_Init();									//大疆遥控器初始化

	led_init();											//led初始化
	key_init();											//按键初始化

	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				//can初始化（电机初始化） 45M/(4+4+1)/5
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,5,CAN_Mode_Normal);				//can初始化（电机初始化） 45M/(4+4+1)/5
   bsp_imu_usart_init(); //串口8
	//Debug_USART_Config();           //通信串口初始化（USART2）
	JSON_USART_Config();           	//主从控通信串口初始化（UART7）
	//bsp_imu_usart_init();
    Referee_USART_Config();
	motor_init();		//电机参数初始化
	fric_PWM_configuration();       //摩擦轮电机pwm初始化   TIM1 新增
	
	TIM1_GIMBAL_Init();
	
  //TIM4_Int_Init(100-1,9000-1);   //测试用时钟
    TIM5_Int_Init(1000-1,18000-1);  //50 hz  
	VPID_Init_All();								//速度pid参数初始化
	APID_Init_All();								//角度pid参数初始化
	stop_chassis_motor();									//让电机保持当前角度（锁死电机）
	ENCODER_Init();
	TIM3_Int_Init(10-1,9000-1);		  //定时器时钟90M，9000，所以90M/9000=10Khz的计数频率，计数10次为1ms  即1khz
	RCC_GetClocksFreq(&get_rcc_clock); 			//查看系统时钟(watch中)
	
	
}

void motor_Init_angle(void)
{
	Kinematics.fric1.target_angular=0;
	Kinematics.fric2.target_angular=0;
	
	Kinematics.pitch.target_angle = 0;
	Kinematics.yaw.target_angle =0;
		
}
