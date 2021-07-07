#ifndef __CAN_H
#define __CAN_H	 
//#include "sys.h"	     

#include "stm32f4xx.h"
	
//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
		


uint8_t CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);//CAN��ʼ��
uint8_t CAN2_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);//CAN2��ʼ��
uint8_t CAN1_Send_CHASSIS_Msg(uint8_t* msg);						//��������
uint8_t CAN1_Send_Trigger_Msg(uint8_t* msg);
uint8_t CAN1_Send_GIMBAL_Msg(uint8_t* msg);
uint8_t CAN1_Receive_Msg(uint8_t *buf);							//��������
#endif

















