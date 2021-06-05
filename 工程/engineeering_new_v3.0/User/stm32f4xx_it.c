/**
  ******************************************************************************
  * @file    Project/USER/stm32f4xx_it.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   中断文件
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include <string.h>
#include "stdint.h"
#include "stm32f4xx_it.h"

#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "can.h"
#include "motor.h"
#include "speed_pid.h"
#include "kinematic.h"
#include "limit_switch.h"
#include "stepper_motor.h"


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/





union {char data[2];uint16_t uivalue;int16_t ivalue;}tmp;

#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)	//解析传回数据*****需增加
{
  	CanRxMsg RxMessage;
    CAN_Receive(CAN1, 0, &RxMessage);
	uint16_t angle;
	int16_t speed,current;//temp
	
	tmp.data[0] = RxMessage.Data[1];
	tmp.data[1] = RxMessage.Data[0];
	angle = tmp.uivalue;//(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
	tmp.data[0] = RxMessage.Data[3];
	tmp.data[1] = RxMessage.Data[2];
	speed = tmp.ivalue;//(RxMessage.Data[2]<<8 | RxMessage.Data[3]);
	current = (RxMessage.Data[4]<<8 | RxMessage.Data[5]);
	//temp = RxMessage.Data[6];
	
	switch(RxMessage.StdId)
	{
		case(CAN_3508Motor1_ID):
			record_motor_callback(&motor1, angle, speed, current);
		break;
		case(CAN_3508Motor2_ID):
			record_motor_callback(&motor2, angle, speed, current);
		break;
		case(CAN_3508Motor3_ID):
			record_motor_callback(&motor3, angle, speed, current);
		break;
		case(CAN_3508Motor4_ID):
			record_motor_callback(&motor4, angle, speed, current);
		break;
		case(CAN_3508Motor5_ID):
			record_motor_callback(&motor5, angle, speed, current);
		break;
		case(CAN_3508Motor6_ID):
	  	record_motor_callback(&motor6, angle, speed, current);
		break;

		case(CAN_LoopBack_ID):
		{
			loopback.motor1_current= RxMessage.Data[0]<<8 | RxMessage.Data[1]<<0;
			loopback.motor2_current= RxMessage.Data[2]<<8 | RxMessage.Data[3]<<0;
			loopback.motor3_current= RxMessage.Data[4]<<8 | RxMessage.Data[5]<<0;
			loopback.motor4_current= RxMessage.Data[6]<<8 | RxMessage.Data[7]<<0;
			
			if(loopback.motor1_current>16384)
				loopback.motor1_current = loopback.motor1_current - 0x10000;
			if(loopback.motor2_current>16384)
				loopback.motor2_current = loopback.motor2_current - 0x10000;
			if(loopback.motor3_current>16384)
				loopback.motor3_current = loopback.motor3_current - 0x10000;
			if(loopback.motor4_current>16384)
				loopback.motor4_current = loopback.motor4_current - 0x10000;
		}
		break;
		default:break;
	}
	
	
}
#endif

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
	
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}
	
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)!= RESET)
	{
		if(0==stepper_motor_left.stop_flag&&0==stepper_motor_left.stop_flag)
		{
			Kinematics.stepper_motor_left.pwm_pulse=0;
			Kinematics.stepper_motor_right.pwm_pulse=0;
			stepper_motor_left.stop_flag++;
			stepper_motor_right.stop_flag++;
		}
		if(1==stepper_motor_left.stop_flag&&1==stepper_motor_left.stop_flag)
		{
			stepper_motor_left.count++;
			stepper_motor_right.count++;
			
			if(300==stepper_motor_left.count&&300==stepper_motor_right.count)
			{
				stepper_motor_left.stop_flag=0;
				stepper_motor_right.stop_flag=0;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)!= RESET)
	{
		if(0==stepper_motor_left.stop_flag&&0==stepper_motor_left.stop_flag)
		{
			Kinematics.stepper_motor_left.pwm_pulse=0;
			Kinematics.stepper_motor_right.pwm_pulse=0;
			stepper_motor_left.stop_flag++;
			stepper_motor_right.stop_flag++;
		}
		if(1==stepper_motor_left.stop_flag&&1==stepper_motor_left.stop_flag)
		{
			stepper_motor_left.count++;
			stepper_motor_right.count++;		
			if(300==stepper_motor_left.count&&300==stepper_motor_right.count)
			{
				stepper_motor_left.stop_flag=0;
				stepper_motor_right.stop_flag=0;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}
///**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @retval None
//  */
//void SVC_Handler(void)
//{}

///**
//  * @brief  This function handles PendSV_Handler exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//void SysTick_Handler(void)
//{}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
