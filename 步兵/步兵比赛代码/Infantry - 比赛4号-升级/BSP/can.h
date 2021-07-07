#ifndef __CAN_H
#define __CAN_H	 
//#include "sys.h"	     

#include "stm32f4xx.h"
	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
		


uint8_t CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);//CAN初始化
uint8_t CAN2_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);//CAN2初始化
uint8_t CAN1_Send_CHASSIS_Msg(uint8_t* msg);						//发送数据
uint8_t CAN1_Send_Trigger_Msg(uint8_t* msg);
uint8_t CAN1_Send_GIMBAL_Msg(uint8_t* msg);
uint8_t CAN1_Receive_Msg(uint8_t *buf);							//接收数据
#endif

















