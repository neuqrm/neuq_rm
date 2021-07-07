/*
*		CAN通信解析回传电机参数
*		存入电机参数结构体
*		发送底盘和云台CAN数据
*/
#include "CAN_receive.h"

struct Motor_data temp_motor_data;
struct Motor_data M3508_ID1_Data	= {0};
struct Motor_data M3508_ID2_Data	= {0};
struct Motor_data M3508_ID3_Data	= {0};
struct Motor_data M3508_ID4_Data	= {0};
struct Motor_data GM6020_Yaw_Data	= {0};
struct Motor_data GM6020_Pitch_Data	= {0};
struct Motor_data M2006_Data	= {0};
CanRxMsg RxMessage_temp;
/***********************************************************/
/*
 * 函数名：CAN1_RX_IRQHandler
 * 描述  ：CAN通信接收数据解析
 * 输入  ：CAN中断信号
 * 范围  ：3508  数据范围 
						给定电流 -16384 ~ 0 ~ +16384
						转子角度	0~8191 等价	0-360°
						转子转速	max=450	RPM
						实际转矩电流
						电机温度	
			   ：GM6020数据范围 
						给定电压 -30000 ~ 0 ~ +30000
						转子角度	0~8191 等价	0-360°
						转子转速	max=350	RPM
						实际转矩电流
						电机温度
			   ：P2006 数据范围 
						给定电流 -10000 ~ 0 ~ +10000
						转子角度	0~8191 等价	0-360°
						转子转速	max=450	RPM
						实际输出转矩
 * 调用  ：CAN1外设
 */	
/************************************************************/
void CAN1_RX_IRQHandler(void)
{
	/*从邮箱中读出报文*/
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage_temp);
	uint8_t angle_temp1,angle_temp2,rotation_temp1,rotation_temp2,\
						current_temp1,current_temp2;
	uint16_t ID_temp;
	ID_temp					= RxMessage_temp.StdId;
	angle_temp1	 		= RxMessage_temp.Data[0];
	angle_temp2			= RxMessage_temp.Data[1];
	temp_motor_data.angle				= 		(angle_temp1<<8) | angle_temp2;
	rotation_temp1	= RxMessage_temp.Data[2];
	rotation_temp2	= RxMessage_temp.Data[3];
	temp_motor_data.rotation		= 		(rotation_temp1<<8) | rotation_temp2;
	current_temp1		= RxMessage_temp.Data[4];
	current_temp2		= RxMessage_temp.Data[5];
	temp_motor_data.current			= 		(current_temp1<<8) | current_temp2 ;
	temp_motor_data.tempature		= 		RxMessage_temp.Data[6];
	switch (ID_temp)
	{
		case CAN_3508_ID1: 		M3508_ID1_Data 	= temp_motor_data;	break;
		case CAN_3508_ID2: 		M3508_ID2_Data 	= temp_motor_data;	break;
		case CAN_3508_ID3: 		M3508_ID3_Data 	= temp_motor_data;	break;
		case CAN_3508_ID4: 		M3508_ID4_Data 	= temp_motor_data;	break;
		case CAN_YAW_ID: 			{GM6020_Yaw_Data = temp_motor_data;	}break;
//													GM6020_Yaw_Data.angle += 2000;
//													if (GM6020_Yaw_Data.angle>=8192)
//													{GM6020_Yaw_Data.angle -= 8192;}break;
		case CAN_PITCH_ID: 		GM6020_Pitch_Data = temp_motor_data;	break;
		case CAN_TRIGGER_ID: 	M2006_Data 	= temp_motor_data;	break;
		default: break;
	}
	
}

void CAN2_RX_IRQHandler(void)
{
	/*从邮箱中读出报文*/
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage_temp);

	
	
}

/*****************Chassic's motor **************/
/*
 * 函数名：CAN_Send_Chassis_Msg/CAN_Send_Gimball_Msg
 * 描述  ：CAN通信报文内容设置,设置一个数据内容为0-7的数据包
 * 输入  ：发送电机电流 or 电压值
 * 范围  ：3508  数据范围 给定电流 -16384 ~ 0 ~ +16384
			   ：GM6020数据范围 给定电压 -30000 ~ 0 ~ +30000
			   ：P2006 数据范围 给定电流 -10000 ~ 0 ~ +10000
 * 调用  ：CAN1外设
 */	

CanTxMsg MsgTEXT;
uint8_t CAN_Send_Chassis_Msg(int16_t motor1 , int16_t motor2 , int16_t motor3 , int16_t motor4)
{	
  uint8_t  mbox;
  uint16_t i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId	=	CAN_CHASSIS_ID_ALL;	 // 标准标识符为0
  TxMessage.ExtId	=	0;	 // 设置扩展标示符（29位）
  TxMessage.IDE		=	CAN_ID_STD;		  // 使用扩展标识符
  TxMessage.RTR		=	CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC		=	8;							 // 发送8字节信息
  TxMessage.Data[0]	=	(motor1 >> 8);
	TxMessage.Data[1]	=	 motor1;
	TxMessage.Data[2]	=	(motor2 >> 8);
	TxMessage.Data[3]	=	 motor2;
	TxMessage.Data[4]	=	(motor3 >> 8);
	TxMessage.Data[5]	=	 motor3;
	TxMessage.Data[6]	=	(motor4 >> 8);
	TxMessage.Data[7]	=	 motor4;
	MsgTEXT = TxMessage;
  mbox		= CAN_Transmit(CAN1, &TxMessage);   
  i	=	0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 0;
  return 1;		

}
/*****************Gimball's motor **************/

uint8_t CAN_Send_Gimball_Msg(int16_t gm_yaw , int16_t gm_pitch , int16_t gm_trigger , int16_t gm_extra)
{	
  uint8_t mbox;
  uint16_t i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId	=	CAN_GIMBAL_ID_ALL;	 // 标准标识符为0
  TxMessage.ExtId	=	0;	 // 设置扩展标示符（29位）
  TxMessage.IDE		=	CAN_ID_STD;		  // 使用扩展标识符
  TxMessage.RTR		=	CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC		=	8;							 // 发送8字节信息
  TxMessage.Data[0]	=	(gm_yaw >> 8);		
	TxMessage.Data[1]	=	 gm_yaw;
	TxMessage.Data[2]	=	(gm_pitch >> 8);
	TxMessage.Data[3]	=	 gm_pitch;
	TxMessage.Data[4]	=	(gm_trigger >> 8);
	TxMessage.Data[5]	=	 gm_trigger;
	TxMessage.Data[6]	=	(gm_extra >> 8);
	TxMessage.Data[7]	=	 gm_extra;
	MsgTEXT = TxMessage;
  mbox		= CAN_Transmit(CAN1, &TxMessage);   
  i	=	0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 0;
  return 1;		
}


uint8_t CAN_Send_Extra_Msg(int16_t extra1 , int16_t extra2 , int16_t extra3 , int16_t extra4)
{	
  uint8_t mbox;
  uint16_t i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId	=	CAN_EXTRA_ALL;	 // 标准标识符为0
  TxMessage.ExtId	=	0;	 // 设置扩展标示符（29位）
  TxMessage.IDE		=	CAN_ID_STD;		  // 使用扩展标识符
  TxMessage.RTR		=	CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC		=	8;							 // 发送8字节信息
  TxMessage.Data[0]	=	(extra1 >> 8);		
	TxMessage.Data[1]	=	 extra1;
	TxMessage.Data[2]	=	(extra2 >> 8);
	TxMessage.Data[3]	=	 extra2;
	TxMessage.Data[4]	=	(extra3 >> 8);
	TxMessage.Data[5]	=	 extra3;
	TxMessage.Data[6]	=	(extra4 >> 8);
	TxMessage.Data[7]	=	 extra4;
	MsgTEXT = TxMessage;
  mbox		= CAN_Transmit(CAN1, &TxMessage);   
  i	=	0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 0;
  return 1;		
}

/**************************END OF FILE************************************/

