#ifndef __CAN_APP_H
#define	__CAN_APP_H

#include "bsp_can.h"

struct Motor_data 
{
	uint16_t angle;				//回传电机角度
	int16_t  rotation;		//回传电机转速
	int16_t  current;			//回传电机电流
	int8_t   tempature;		//回传电机温度
};											//3508反馈报文

extern struct Motor_data M3508_ID1_Data;
extern struct Motor_data M3508_ID2_Data;
extern struct Motor_data M3508_ID3_Data;
extern struct Motor_data M3508_ID4_Data;
extern struct Motor_data GM6020_Yaw_Data;
extern struct Motor_data GM6020_Pitch_Data;
extern struct Motor_data M2006_Data;

enum Motor_ID
{
	  CAN_CHASSIS_ID_ALL = 0x200,
    CAN_3508_ID1 = 0x201,
    CAN_3508_ID2 = 0x202,
    CAN_3508_ID3 = 0x203,
    CAN_3508_ID4 = 0x204,

    CAN_YAW_ID = 0x208,
    CAN_PITCH_ID = 0x206,
    CAN_TRIGGER_ID = 0x207,
    CAN_GIMBAL_ID_ALL = 0x1FF,
	
		CAN_EXTRA_ALL = 0x2FF,
};

uint8_t CAN_Send_Chassis_Msg(int16_t motor1 , int16_t motor2 , int16_t motor3 , int16_t motor4);
uint8_t CAN_Send_Gimball_Msg(int16_t gm_yaw , int16_t gm_pitch , int16_t gm_trigger , int16_t gm_extra);	
uint8_t CAN_Send_Extra_Msg(int16_t extra1 , int16_t extra2 , int16_t extra3 , int16_t extra4);

#endif

