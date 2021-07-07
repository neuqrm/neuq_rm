
#include "debug.h"
#include "motor.h"
#include "pid.h"

float Data_0=0,Data_1=0,Data_2=0,Data_3=0,\
				Data_4=0,Data_5=0,Data_6=0,Data_7=0,Data_8=0,\
				Data_9=0,Data_10=0,Data_11=0,Data_12=0,\
				Data_13=0,Data_14=0;


/****************匿名数传发送数据*******************************************/
/*
 *发送2字节数据 int16型 -32767~+32767
 */
void DEBUG_Anony16Send(uint8_t id,int16_t *pst)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[46];//数据最长40字节 int16数据20个
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = ANONY_ADDR;
	data_to_send[count++] = id;
	data_to_send[count++] = 0;
	data_to_send[count++] = (unsigned char)pst[0]; 
	data_to_send[count++] = (unsigned char)(pst[0]>>8);  //高8位
	data_to_send[count++] = (unsigned char)pst[1];  			//低8位
	data_to_send[count++] = (unsigned char)(pst[1]>>8);
	data_to_send[count++] = (unsigned char)pst[2];
	data_to_send[count++] = (unsigned char)(pst[2]>>8);
	data_to_send[count++] = (unsigned char)pst[3];
	data_to_send[count++] = (unsigned char)(pst[3]>>8);
	data_to_send[count++] = (unsigned char)pst[4];
	data_to_send[count++] = (unsigned char)(pst[4]>>8);
	data_to_send[count++] = (unsigned char)pst[5];
	data_to_send[count++] = (unsigned char)(pst[5]>>8);
	data_to_send[count++] = (unsigned char)pst[6];
	data_to_send[count++] = (unsigned char)(pst[6]>>8);
	data_to_send[count++] = (unsigned char)pst[7];
	data_to_send[count++] = (unsigned char)(pst[7]>>8);
	data_to_send[count++] = (unsigned char)pst[8];
  data_to_send[count++] = (unsigned char)(pst[8]>>8);
	data_to_send[count++] = (unsigned char)pst[9];
	data_to_send[count++] = (unsigned char)(pst[9]>>8);
	data_to_send[count++] = (unsigned char)pst[10];
	data_to_send[count++] = (unsigned char)(pst[10]>>8);
	//小端模式 低字节在前 高字节在后 一次最多10个数据
	data_to_send[3] = count - 4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	for (i=0;i<count;i++)
		DEBUG_SendByte(data_to_send[i]);
}

/*
 *发送4字节数据 int32型 非常大 可以除以相应10n次方，转换为小数
 */
void DEBUG_Anony32Send(uint8_t id,int32_t *pst)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[66];//数据最长60字节 int16数据40个
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = ANONY_ADDR;
	data_to_send[count++] = id;
	data_to_send[count++] = 0;
	data_to_send[count++] = (unsigned char)pst[0]; 			//低8位
	data_to_send[count++] = (unsigned char)(pst[0]>>8);  //高8位
	data_to_send[count++] = (unsigned char)(pst[0]>>16); //高16位
	data_to_send[count++] = (unsigned char)(pst[0]>>24); //高24位
	data_to_send[count++] = (unsigned char)pst[1];  			
	data_to_send[count++] = (unsigned char)(pst[1]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[2];
	data_to_send[count++] = (unsigned char)(pst[2]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[3];
	data_to_send[count++] = (unsigned char)(pst[3]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[4];
	data_to_send[count++] = (unsigned char)(pst[4]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[5];
	data_to_send[count++] = (unsigned char)(pst[5]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[6];
	data_to_send[count++] = (unsigned char)(pst[6]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[7];
	data_to_send[count++] = (unsigned char)(pst[7]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[8];
  data_to_send[count++] = (unsigned char)(pst[8]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[9];
	data_to_send[count++] = (unsigned char)(pst[9]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	data_to_send[count++] = (unsigned char)pst[10];
	data_to_send[count++] = (unsigned char)(pst[10]>>8);
	data_to_send[count++] = (unsigned char)(pst[0]>>16);
	data_to_send[count++] = (unsigned char)(pst[0]>>24);
	//小端模式 低字节在前 高字节在后 一次最多10个数据
	data_to_send[3] = count - 4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	for (i=0;i<count;i++)
		DEBUG_SendByte(data_to_send[i]);
}
/****************************匿名数传参数接收返回*************************/
void DEBUG_Anonyreturn(uint8_t *pst)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[9];//参数返回帧格式
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = STM32_ADDR;
	data_to_send[count++] = RETURN_ID;
	data_to_send[count++] = 3;
	data_to_send[count++] = pst[0];
	data_to_send[count++] = pst[1];
	data_to_send[count++] = pst[2];
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	for (i=0;i<count;i++)
		DEBUG_SendByte(data_to_send[i]);
}

/*****************************匿名数传参数接收*****************************/
Debug_receive Debug_data_pack;
void DEBUG_Data_solve(uint8_t ucData)
{
	//static uint8_t ucRxBuffer[20][12];
	static uint8_t ucRxBuffer[36];
	static uint8_t ucRxCnt = 0;
	uint8_t sumcheck = 0,addcheck = 0,i; 
	uint8_t return_data[3];
	ucRxBuffer[ucRxCnt++] = ucData;
	if (ucRxBuffer[0]!=ANONY_HEAD) //数据头不对，则重新开始寻找0xAA数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<12) {return;}//数据不满12个，则返回
	else
	{

			for (i=0;i<ucRxBuffer[3]+4;i++)
		{
			sumcheck += ucRxBuffer[i];
			addcheck += sumcheck;
		}
		if(sumcheck == ucRxBuffer[10]&&addcheck == ucRxBuffer[11])
		{
			memcpy(&Debug_data_pack,&ucRxBuffer[4],6);	
			return_data[1] = ucRxBuffer[10];
			return_data[2] = ucRxBuffer[11];
			return_data[0] = STM32_WRITE;
			DEBUG_Anonyreturn(return_data);
			ucRxCnt=0;
		}
		switch (Debug_data_pack.PAR_ID)
		{
			case 0:
			{
				Data_0 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_yaw_speed.p = Data_0; //220//135
			}break;
			case 1:
			{
				Data_1 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_yaw_speed.i = Data_1; //6
			}break;
			case 2:
			{
				Data_2 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_yaw_speed.d = Data_2; //0
			}break;
			case 3:
			{
				Data_3 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_wheel_speed[0].p = Data_3;
				pid_wheel_speed[1].p = Data_3;
				pid_wheel_speed[2].p = Data_3;
				pid_wheel_speed[3].p = Data_3;
			}break;
			case 4:
			{
				Data_4 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_wheel_speed[0].i = Data_4;
				pid_wheel_speed[1].i = Data_4;
				pid_wheel_speed[2].i = Data_4;
				pid_wheel_speed[3].i = Data_4;
			}break;
			case 5:
			{
				Data_5 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_wheel_speed[0].d = Data_5;
				pid_wheel_speed[1].d = Data_5;
				pid_wheel_speed[2].d = Data_5;
				pid_wheel_speed[3].d = Data_5;
			}break;
			case 6:
			{
				Data_6 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_wheel_current[0].p = Data_6;
				pid_wheel_current[1].p = Data_6;
				pid_wheel_current[2].p = Data_6;
				pid_wheel_current[3].p = Data_6;
			}break;
			case 7:
			{
				Data_7 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_wheel_current[0].i = Data_7;
				pid_wheel_current[1].i = Data_7;
				pid_wheel_current[2].i = Data_7;
				pid_wheel_current[3].i = Data_7;
			}break;
			case 8:
			{
				Data_8 = ((float)Debug_data_pack.PAR_VAL/100);
				pid_wheel_current[0].d = Data_8;
				pid_wheel_current[1].d = Data_8;
				pid_wheel_current[2].d = Data_8;
				pid_wheel_current[3].d = Data_8;
			}break;
			case 9:
			{
				Data_9 = ((float)Debug_data_pack.PAR_VAL/1000);
				pid_yaw_speed.p = Data_9; //1
			}break;
			case 10:
			{
				Data_10 = ((float)Debug_data_pack.PAR_VAL/1000);
				pid_yaw_speed.i = Data_10; //1
			}break;
			case 11:
			{
				Data_11 = ((float)Debug_data_pack.PAR_VAL/1000);
				pid_yaw_speed.d = Data_11; //0
			}break;
			case 12:
			{
				Data_12 = ((float)Debug_data_pack.PAR_VAL/1000);
				pid_yaw_angle.p = Data_12; //1
			}break;
			case 13:
			{
				Data_13 = ((float)Debug_data_pack.PAR_VAL/1000);
				pid_yaw_angle.i = Data_13; //1
			}break;
			case 14:
			{
				Data_14 = ((float)Debug_data_pack.PAR_VAL/1000);
				pid_yaw_angle.d = Data_14; //0
			}break;
			default:break;
		}
	}
}
/**************************************************************************/


