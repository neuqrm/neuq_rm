
#include "imu.h"
#include "bsp_imu_usart.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;

FAcc			IMUAcc = {0};
FGyro			IMUGyro = {0};
FAngle		IMUAngle = {0};

char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置

/******************************************************
* @fn Init_IMU_Struct_Data
*
* @brief 初始IMU回传数结构体
* @pData None
* @return None.
* @note 在主函数开始前调用一次
*/

void Init_IMU_Struct_Data(void)
{
		memset(&stcTime,0,8);
		memset(&stcAcc,0,8);
		memset(&stcGyro,0,8);
		memset(&stcAngle,0,8);
		memset(&stcMag,0,8);
		memset(&stcDStatus,0,8);
		memset(&stcPress,0,8);
		memset(&stcLonLat,0,8);
		memset(&stcGPSV,0,8);
		memset(&stcQ,0,8);
}
/******************************************************
* @fn IMU_CALSW_Set
*
* @brief 设置IMU校准模式 
* @pData None
* @return None.
* @note 只需调用一次
*/

void IMU_CALSW_Set(void)
{
	IMU_SendString(ACCCALSW);
}

/******************************************************
* @fn IMU_SAVE_Set(void)
*
* @brief 初始IMU回传数结构体
* @pData None
* @return None.
* @note 只需调用一次
*/

void IMU_SAVE_Set(void)
{
	IMU_SendString(SAVACALSW);
}
/******************************************************
* @fn IMU_Data_Solve
*
* @brief 解析接收数据 放入对应结构体中
* @pData 接收数据的指针
* @return None.
* @note USART8中断中调用
*/
void IMU_Data_Solve(uint8_t ucData)
{
	static uint8_t ucRxBuffer[35];
	static uint8_t ucRxCnt = 0;	
	float Yaw_radsp=0;
	ucRxBuffer[ucRxCnt++] = ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	
			{
				memcpy(&stcTime,&ucRxBuffer[2],8);
			}break;
			case 0x51:	
			{
				memcpy(&stcAcc,&ucRxBuffer[2],8);
				IMUAcc.Ax = (float)stcAcc.a[0]/32768*16;
				IMUAcc.Ay = (float)stcAcc.a[1]/32768*16;
				IMUAcc.Az = (float)stcAcc.a[2]/32768*16;
			}break;
			case 0x52:	
			{
				memcpy(&stcGyro,&ucRxBuffer[2],8);
				IMUGyro.Wx = (float)stcGyro.w[0]/32768*2000;
				IMUGyro.Wy = (float)stcGyro.w[1]/32768*2000;
				Yaw_radsp = (float)stcGyro.w[2]/32768*2000;
				if (((Yaw_radsp-IMUGyro.Wz)>50)||(Yaw_radsp-IMUGyro.Wz)<-50)
				{}
					else 
				IMUGyro.Wz = Yaw_radsp;
			}break;
			case 0x53:	
			{
				memcpy(&stcAngle,&ucRxBuffer[2],8);
				IMUAngle.imuRoll 	= (float)stcAngle.Angle[0]/32768*180;
				IMUAngle.imuPitch = (float)stcAngle.Angle[1]/32768*180;
				IMUAngle.imuYaw 	= (float)stcAngle.Angle[2]/32768*180;
			}break;
			case 0x54:	
			{
				memcpy(&stcMag,&ucRxBuffer[2],8);
			}break;
			case 0x55:	
			{
				memcpy(&stcDStatus,&ucRxBuffer[2],8);
			}break;
			case 0x56:	
			{
				memcpy(&stcPress,&ucRxBuffer[2],8);
			}break;
			case 0x57:	
			{
				memcpy(&stcLonLat,&ucRxBuffer[2],8);
			}break;
			case 0x58:	
			{
				memcpy(&stcGPSV,&ucRxBuffer[2],8);
			}break;
			case 0x59:	
			{
				memcpy(&stcQ,&ucRxBuffer[2],8);
			}break;
		}//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
		ucRxCnt=0;//清空缓存区
	}
}
