
#include "json.h"

Chassis_Rpm_stc  	Json_Chassis_Rpm		=	{0};
Gimbal_Yaw_stc  	Json_Gimbal_Yaw		=	{0};
Gimbal_Pitch_stc 	Json_Gimbal_Pitch	=	{0};

/*
*JSON串口中断数据处理函数 进行数据处理
*无输出 使用三个全局变量
*Jason_Receive_Flag JSON串口工作标志 串口工作状态下无法解析数据
*Jason_Data_Flag JSON数据包类型标志
********************************************************************/
char Json_Buffer[MAX_LENGTH]; 
uint8_t Jason_Receive_Flag = JASON_STOP,Jason_Data_Flag = 0;
void JASON_Data_Solve(uint8_t ucTemp)
{
	static uint8_t json_count = 0;
	static char ReceiveBuffer[MAX_LENGTH];
	if(ucTemp == '*')	//开始标志位
	{
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //清空接收缓冲区,防止如果没收到结束标志位，缓冲区未清零的情况
		json_count=0;
		Jason_Receive_Flag = JASON_START;	//开始标志置一
	}
	else if(ucTemp == ';')	//结束标志位
	{
		json_count=0;
		//标志位置一，标识收到命令
		Jason_Data_Flag = JASON_GIMBALL_ANGLE;
		Jason_Receive_Flag = JASON_STOP; //结束标志置一
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//清空缓冲区，防止上一次的数据剩余导致乱码
		strcpy(Json_Buffer,ReceiveBuffer);	//接收缓冲区给json缓冲区
		memset(ReceiveBuffer,    0, sizeof(ReceiveBuffer)); //清空接收缓冲区
	}//********************************************
	else if(ucTemp == '?')	//结束标志位
	{
		json_count=0;
		//标志位置一，标识收到命令
		Jason_Data_Flag = JASON_GIMBALL_ACTION;
		Jason_Receive_Flag = JASON_STOP; //结束标志置一
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//清空缓冲区，防止上一次的数据剩余导致乱码
		strcpy(Json_Buffer,ReceiveBuffer);	//接收缓冲区给json缓冲区
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //清空接收缓冲区	
	}//********************************************
	else if(ucTemp == '!')	//结束标志位
	{
		json_count=0;
		//标志位置一，标识收到命令
		Jason_Data_Flag = JASON_CHASSIC_SPEED;
		Jason_Receive_Flag = JASON_STOP; //结束标志置一
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//清空缓冲区，防止上一次的数据剩余导致乱码
		strcpy(Json_Buffer,ReceiveBuffer);	//接收缓冲区给json缓冲区
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //清空接收缓冲区
	}//********************************************
	else if(ucTemp == '.')	//结束标志位
	{
		json_count=0;
		//标志位置一，标识收到命令
		Jason_Data_Flag = JASON_GIMBALL_SHOOT;
		Jason_Receive_Flag = JASON_STOP; //结束标志置一
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//清空缓冲区，防止上一次的数据剩余导致乱码
		strcpy(Json_Buffer,ReceiveBuffer);	//接收缓冲区给json缓冲区
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //清空接收缓冲区	
	}
	else if(Jason_Receive_Flag == JASON_START)
	{
		ReceiveBuffer[json_count++] = ucTemp;
	}
}

/**
  *@brief 发送云台信息
  */
void Json_Gimball_Angle_Send(void)   
{
  json_t *root;
	char *out;           
	root = json_pack("[{sfsf}[ff]]",\
					  "yaw_angular", (Motion_Gimbal_Yaw.Yaw_rad),\
						"pitch_angular", (Motion_Gimbal_Pitch.Pitch_rad),\
						(Motion_Gimbal_Yaw.Yaw_rad),\
						(Motion_Gimbal_Pitch.Pitch_rad));   //(int),
	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);
}
/**
  *@brief 解析云台角度
  */
void Json_Gimball_Angle_Reslove(void)
{
	json_t *root;
	json_t *gimbal_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(Json_Buffer,0,&error);
	gimbal_obj = json_object_get( root, "gimbal" );
	item_obj = json_array_get( gimbal_obj, 0 );
  Json_Gimbal_Yaw.Yaw_rad = 1.0f*json_integer_value(item_obj); //10000;
	item_obj = json_array_get( gimbal_obj, 1 );
	Json_Gimbal_Pitch.Pitch_rad = 1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(gimbal_obj);
	json_decref(root);	
}
