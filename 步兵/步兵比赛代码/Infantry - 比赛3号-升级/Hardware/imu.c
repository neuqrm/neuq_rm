
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

char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//������ٶ�У׼ģʽ
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//���浱ǰ����

/******************************************************
* @fn Init_IMU_Struct_Data
*
* @brief ��ʼIMU�ش����ṹ��
* @pData None
* @return None.
* @note ����������ʼǰ����һ��
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
* @brief ����IMUУ׼ģʽ 
* @pData None
* @return None.
* @note ֻ�����һ��
*/

void IMU_CALSW_Set(void)
{
	IMU_SendString(ACCCALSW);
}

/******************************************************
* @fn IMU_SAVE_Set(void)
*
* @brief ��ʼIMU�ش����ṹ��
* @pData None
* @return None.
* @note ֻ�����һ��
*/

void IMU_SAVE_Set(void)
{
	IMU_SendString(SAVACALSW);
}
/******************************************************
* @fn IMU_Data_Solve
*
* @brief ������������ �����Ӧ�ṹ����
* @pData �������ݵ�ָ��
* @return None.
* @note USART8�ж��е���
*/
void IMU_Data_Solve(uint8_t ucData)
{
	static uint8_t ucRxBuffer[35];
	static uint8_t ucRxCnt = 0;	
	float Yaw_radsp=0;
	ucRxBuffer[ucRxCnt++] = ucData;
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
		}//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
		ucRxCnt=0;//��ջ�����
	}
}
