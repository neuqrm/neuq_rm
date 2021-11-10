/**
  ******************************************************************************
  * @file    Project/USER/main.c 
  * @author  Q&L
  * @version V1.0.0
  * @date    2019
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
	
	
#include "stm32f4xx.h"
#include "bsp_advance_tim.h"
#include "bsp_basic_tim.h"
#include "bsp_key.h" 
#include "bsp_SysTick.h"
#include "bsp_can.h"
#include "bsp_led.h"
#include "bsp_power.h"
#include "bsp_dbus.h"
#include "bsp_debug_usart.h"
#include "bsp_supercap_usart.h"
#include "bsp_referee_usart.h" 
#include "bsp_imu_usart.h"

#include "CAN_receive.h"
#include "imu.h"
#include "filter.h"

#include "remote.h"
#include "motor.h"
#include "motion.h"
#include "referee.h"
#include "limit.h"

//extern __IO uint16_t ChannePulse;
	 __IO uint16_t ChannePulse1 = 1000;
	 __IO uint16_t ChannePulse2 = 1200;
	 __IO uint16_t ChannePulse3 = 1400;
	 __IO uint16_t ChannePulse4 = 1600;
	 __IO uint16_t ChannePulse5 = 1800;
	 __IO uint16_t ChannePulse6 = 2000; 
	 
	int16_t current1 = 0;
	int16_t current2 = 2000;
	int16_t current3 = 3000;
	int16_t current4 = -3000;
	int16_t current5 = -2000;
	int16_t current6 = -1000;
	char PowerStr[20];


	RCC_ClocksTypeDef RCC_clocks;

	char FLAG = '2';
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void) 
 {
	/* ��ʼ������GPIO */
  Key_GPIO_Config();
	SysTick_Init();
	LED_GPIO_Config();
	POWER_GPIO_Config();
	Dji_Remote_Init();
  /* ��ʼ���߼����ƶ�ʱ��������PWMģʽ��ʹ��ͨ��1\2\3\4��� */
	TIM1_Configuration();
	TIM6_Configuration();
	TIM8_Configuration();
	CAN1_Config();
	CAN2_Config();
	Debug_USART_Config();
	CAP_USART_Config();
	
	Referee_USART_Config();
  /*APP���йع��ܺ�����ʼ��*/
	IMU_USART_Config();
	Init_Referee_Struct_Data();
	Motor_PID_init();
	Init_IMU_Struct_Data();
	/*�ϻ�ǰ��ʼ��*/
	IMU_SendString(ACCCALSW);
	Delay_ms(1000);
	Delay_ms(1000);
	IMU_SendString(SAVACALSW);
  while(1)
  {  
		uint8_t i;
		int16_t powerT[10];
		float power,powerbuff,current;
		DJI_Remote_Control();
		Motion_Ctrl();
		LimitCtrl();
		//Motot_Control();
		//RCC_GetClocksFreq(&RCC_clocks);
		power = Get_Chassis_Power();	
		powerT[0] = power*100;
		powerbuff = Get_Chassis_PowerBuff();
		powerT[1] = powerbuff*100;
		current = Get_Chassis_Current();
		powerT[2] = current*100;
		powerT[3] = 6000;
		powerT[3] = 8000;
		if (basic_count%20==0)
			Referee_Data_Send(0x0103);
		if (basic_count%4==0)
			CAP_SendByte(SuperCapVoltage);
		//DEBUG_Anony16Send(ANONY_ID1,powerT);//���͹���
	}   
}



/*********************************************END OF FILE**********************/

