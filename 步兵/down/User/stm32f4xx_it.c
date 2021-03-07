/**
  ******************************************************************************
  * @file    Project/USER/stm32f4xx_it.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   �ж��ļ�
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include <string.h>
#include "stdint.h"
#include "stm32f4xx_it.h"
#include <jansson.h>
#include "json.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "can.h"
#include "motor.h"
#include "speed_pid.h"
#include "kinematic.h"
#include "imuReader.h"
#include "bsp_imu_usart.h"


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/
	/**
  * @brief  ����2�жϷ�����
  * @param  None
  * @retval None
  */
/*void DEBUG_USART_IRQHandler(void)
{
	uint8_t ucTemp;
//	uint8_t mbox;
//	uint16_t i;
	if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)//�����жϣ��������ǿգ���ʾ�ѽ��յ�����
  {
		ucTemp = USART_ReceiveData( DEBUG_USART );
		switch(ucTemp)
		{
			case 'f':
			{
				BaseVel_To_WheelVel(0, 50, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 'b':
			{
				BaseVel_To_WheelVel(0, -50, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 'l':
			{
				BaseVel_To_WheelVel(-50, 0, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 'r':
			{
				BaseVel_To_WheelVel(50, 0, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 's':
			{
				BaseVel_To_WheelVel(0, 0, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			default:
				break;
		}
	}
	if(USART_GetITStatus(DEBUG_USART, USART_IT_TXE) != RESET)//�������ѿգ���ʾ���Լ�����������
  {
		
  }
	USART_ClearITPendingBit(DEBUG_USART,USART_IT_ORE);
}*/

//json�ַ������ձ�־λ����һλΪ
__IO uint8_t flag_json_recieve = 0x00;
const uint8_t flag_json_start   = 0x01;
const uint8_t flag_json_start_mask   = 0xFE;
const uint8_t flag_json_stop    = 0x02;
const uint8_t flag_json_stop_mask    = 0xFD;
//���ջ�����__IO


char receiveBuffer[MAX_LENGTH];
char json_Buffer[MAX_LENGTH];
uint8_t flag_command_recieved = 0;
uint8_t flag_command_recieved1 = 0;
uint8_t flag_command_recieved2 = 0;
uint8_t flag_command_recieved3 = 0;
uint8_t flag_command_recieved4 = 0;
uint8_t flag_command_recieved5 = 0;


void JSON_USART_IRQHandler(void)
{
	uint8_t ucTemp;
	static uint8_t json_count=0;
	if(USART_GetITStatus(JSON_USART, USART_IT_RXNE) != RESET)//�����жϣ��������ǿգ���ʾ�ѽ��յ�����
  {
		ucTemp = USART_ReceiveData( JSON_USART );
		if(ucTemp == '*')	//��ʼ��־λ
		{
			memset(receiveBuffer, 0, sizeof(receiveBuffer)); //��ս��ջ�����,��ֹ���û�յ�������־λ��������δ��������
			json_count=0;
			flag_json_recieve |= flag_json_start;	//��ʼ��־��һ
			flag_json_recieve &= flag_json_stop_mask;	//������־����
		}
		else if(ucTemp == ';')	//������־λ
		{
			json_count=0;
			//��־λ��һ����ʶ�յ�����
			flag_command_recieved = 1;
			flag_json_recieve |= flag_json_stop; //������־��һ
			flag_json_recieve &= flag_json_start_mask;	//��ʼ��־����
			//printf("%s",receiveBuffer);
			memset(json_Buffer, 0, sizeof(json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
			strcpy(json_Buffer,receiveBuffer);	//���ջ�������json������
			memset(receiveBuffer,    0, sizeof(receiveBuffer)); //��ս��ջ�����
			
		}//********************************************
			else if(ucTemp == '?')	//������־λ
		{
			json_count=0;
			//��־λ��һ����ʶ�յ�����
			flag_command_recieved1 = 1;
			flag_json_recieve |= flag_json_stop; //������־��һ
			flag_json_recieve &= flag_json_start_mask;	//��ʼ��־����
			//printf("%s",receiveBuffer);
			memset(json_Buffer, 0, sizeof(json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
			strcpy(json_Buffer,receiveBuffer);	//���ջ�������json������
			memset(receiveBuffer, 0, sizeof(receiveBuffer)); //��ս��ջ�����
			
		}//********************************************
		else if(ucTemp == '!')	//������־λ
		{
			json_count=0;
			//��־λ��һ����ʶ�յ�����
			flag_command_recieved2 = 1;
			flag_json_recieve |= flag_json_stop; //������־��һ
			flag_json_recieve &= flag_json_start_mask;	//��ʼ��־����
			//printf("%s",receiveBuffer);
			memset(json_Buffer, 0, sizeof(json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
			strcpy(json_Buffer,receiveBuffer);	//���ջ�������json������
			memset(receiveBuffer, 0, sizeof(receiveBuffer)); //��ս��ջ�����
			
		}//********************************************
			else if(ucTemp == '.')	//������־λ
		{
			json_count=0;
			//��־λ��һ����ʶ�յ�����
			flag_command_recieved3 = 1;
			flag_json_recieve |= flag_json_stop; //������־��һ
			flag_json_recieve &= flag_json_start_mask;	//��ʼ��־����
			//printf("%s",receiveBuffer);
			memset(json_Buffer, 0, sizeof(json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
			strcpy(json_Buffer,receiveBuffer);	//���ջ�������json������
			memset(receiveBuffer, 0, sizeof(receiveBuffer)); //��ս��ջ�����
			
		}
		else if(ucTemp == '>')	//������־λ
		{
			json_count=0;
			//��־λ��һ����ʶ�յ�����
			flag_command_recieved4 = 1;
			flag_json_recieve |= flag_json_stop; //������־��һ
			flag_json_recieve &= flag_json_start_mask;	//��ʼ��־����
			//printf("%s",receiveBuffer);
			memset(json_Buffer, 0, sizeof(json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
			strcpy(json_Buffer,receiveBuffer);	//���ջ�������json������
			memset(receiveBuffer, 0, sizeof(receiveBuffer)); //��ս��ջ�����
			
		}//********************************************
		else if(ucTemp == '+')	//������־λ  
		{
			json_count=0;
			//��־λ��һ����ʶ�յ�����
			flag_command_recieved5 = 1;
			flag_json_recieve |= flag_json_stop; //������־��һ
			flag_json_recieve &= flag_json_start_mask;	//��ʼ��־����
			//printf("%s",receiveBuffer);
			memset(json_Buffer, 0, sizeof(json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
			strcpy(json_Buffer,receiveBuffer);	//���ջ�������json������
			memset(receiveBuffer, 0, sizeof(receiveBuffer)); //��ս��ջ�����
			
		}
		
		
		else if(((flag_json_recieve & flag_json_start) == flag_json_start))
		{
			receiveBuffer[json_count++] = ucTemp;
		}

	}
	if(USART_GetITStatus(JSON_USART, USART_IT_TXE) != RESET)//�������ѿգ���ʾ���Լ�����������
  {
		
  }
	USART_ClearITPendingBit(JSON_USART,USART_IT_ORE);
}



union {char data[2];uint16_t uivalue;int16_t ivalue;}tmp;

#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)	//������������*****������
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
		case(CAN_TRIGGER_ID):
			record_motor_callback(&motor5, angle, speed, current);
		break;
		case(CAN_GIMBAL_Y_ID):
		  record_motor_callback(&gimbal_y, angle, speed, current);
		break;
		case(CAN_GIMBAL_P_ID):
		  record_motor_callback(&gimbal_p, angle, speed, current);
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
extern IMU_DATA imu_data;
void IMU_USART_IRQHandler(void){
	
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	

		unsigned char ucData = USART_ReceiveData(USART6);
		ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
		if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
		{
			ucRxCnt=0;
			return;
		}
		if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
		else
		{
			switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
			{
				case 0x50:	
					
				break;//time
				case 0x53:	
					imu_data.yaw = ((ucRxBuffer[YAWH]<<8)|ucRxBuffer[YAWL])*0.0000958737992428;
				break;//angle
				case 0x52:
					imu_data.yaw_angular=(short)((ucRxBuffer[YAWH_angular]<<8)|ucRxBuffer[YAWL_angular])*0.000030517578125*2000;
				  imu_data.pitch_angular=(short)((ucRxBuffer[PITCHH_angular]<<8)|ucRxBuffer[PITCHL_angular])*0.000030517578125*2000;
				break;
				case 0x51:
					imu_data.ax=(short)((ucRxBuffer[3]<<8)|ucRxBuffer[2])*0.000030517578125*16*9.8;
					imu_data.ay=(short)((ucRxBuffer[5]<<8)|ucRxBuffer[4])*0.000030517578125*16*9.8;
			  	imu_data.az=(short)((ucRxBuffer[7]<<8)|ucRxBuffer[6])*0.000030517578125*16*9.8;
				break;
					
				
				default:break;			
			}
			ucRxCnt=0;//��ջ�����
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
