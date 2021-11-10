

#include "remote.h"

Chassis_Rpm_stc			Remote_Chassis_Rpm	=	{0};
Gimbal_Yaw_stc			Remote_Gimbal_Yaw		=	{0};
Gimbal_Pitch_stc		Remote_Gimbal_Pitch	=	{0};
Gimbal_Trigger_stc	Remote_Gimbal_Trigger	=	{0};


/**************************************************
ս������DJIң��������
**************************************************/
int16_t Debug_dataremote[10]={0};
uint8_t AIMMODE	=	REMOTEAIM;
uint8_t CHASSISMODE	=	NORMALCHASSIS;
void DJI_Remote_Control(void)
{
	if (DJI_Action_Flag != DJI_Action_Change)
	{
		DJI_Remote_Chassis();
		DJI_Remote_Gimball();
		DJI_Remote_Action();
	}
	else
	{
		LASER_ON;
		PC_Remote_Control();
	}
	Debug_dataremote[0]=Remote_Chassis_Rpm.Xrpm;
	Debug_dataremote[1]=Remote_Chassis_Rpm.Yrpm;
	Debug_dataremote[2]=Remote_Chassis_Rpm.Zrpm;
	Debug_dataremote[3]=Remote_Gimbal_Yaw.Yaw_rad;
	Debug_dataremote[4]=Remote_Gimbal_Yaw.Yaw_rpm;
	Debug_dataremote[5]=Remote_Gimbal_Pitch.Pitch_rad;
	Debug_dataremote[6]=Remote_Gimbal_Pitch.Pitch_rpm;
	Debug_dataremote[7]=Remote_Gimbal_Trigger.Fric_rpm;
	Debug_dataremote[8]=Remote_Gimbal_Trigger.Trigger_rad;
	Debug_dataremote[9]=Remote_Gimbal_Trigger.Trigger_rpm;
	//DEBUG_Anony16Send(ANONY_ID2,Debug_dataremote);
}

/**************************************************
ս������PCң��������
**************************************************/
void PC_Remote_Control(void)
{
		PC_Remote_Chassis();
		PC_Remote_Gimball();
		PC_Remote_Action();
}
/***************************************************
ң�������̿��ƴ���
���ܣ���ҡ�˻ش�����ֵת��Ϊ���̵��˶�
���ã����̿��ƴ��롢ң������������
��ʽ���ٶȿ���
������� ����Ϊ�����˶�
����ƽ���˶�
������ת�˶�
***************************************************/

void DJI_Remote_Chassis(void)
{
	float tempX=0,tempY=0,tempZ=0;
	tempX = (rc.ch1-DJI_CHMID)/DJI_XVAL;
	tempY = (rc.ch0-DJI_CHMID)/DJI_YVAL;
	tempZ = (rc.ch2-DJI_CHMID)/DJI_ZVAL;
	Remote_Chassis_Rpm.Xrpm = tempX;
	Remote_Chassis_Rpm.Yrpm = tempY;
	Remote_Chassis_Rpm.Zrpm = tempZ;
}
/**************************************************
ң������̨���ƴ���
���ܣ���ҡ�˺Ͳ��ֻش�����ֵת��Ϊ��̨���˶�
���ã���̨���ƴ��롢ң������������
��ʽ���Ƕȿ��� or �ٶȿ���
������� ����Ϊ��̨�˶�
����pitch�˶�
����yaw�˶�
**************************************************/
int16_t LookB = 0;
void DJI_Remote_Gimball(void)
{
	float temp_Yangle=YAW_RESETANGLE,temp_Pangle=PITCH_RESETANGLE,temp_Yspeed=0,temp_Pspeed=0;
	temp_Yangle = (rc.sw - DJI_MIN)*(YAW_MAXRAD - YAW_MINRAD)/(DJI_MAX - DJI_MIN)+YAW_MINRAD;
	temp_Pangle = (rc.ch3 - DJI_MIN)*(PITCH_MAXRAD - PITCH_MINRAD)/(DJI_MAX - DJI_MIN)+PITCH_MINRAD;
	temp_Yspeed = (rc.sw-DJI_CHMID)*MAXSPEEDY/DJI_MID;
	temp_Pspeed = (rc.ch3 - DJI_CHMID)*MAXSPEEDP/DJI_MID;
	Remote_Gimbal_Yaw.Yaw_rad = temp_Yangle;
	Remote_Gimbal_Yaw.Yaw_rpm = temp_Yspeed;
	Remote_Gimbal_Pitch.Pitch_rad = temp_Pangle;
	Remote_Gimbal_Pitch.Pitch_rpm = temp_Pspeed;
}
/**************************************************
ս����������
���ܣ������Ƹ����ݣ��ж�ս������
���ã���̨�����̿��ƴ��룬ң������������
��ʽ���߼�����
������� ����Ϊս������
ͨ��4~5	S1~2 ��S1 ��S2
��	=	1��		��	=	3��		��	=	2��
#define DJI_Action_Flag		((rc.s1<<4)|rc.s2)	//�Ƹ�ѡ����̨����
#define	DJI_Action_Reset			((3<<4)|3)	//��λ״̬ �޶���
#define	DJI_Action_Change			((3<<4)|1)	//ң�����ͼ����л�
#define	DJI_Action_Brake			((3<<4)|2)		//����ɲ��
#define	DJI_Action_Fric				((1<<4)|3)	//��Ħ����
#define	DJI_Action_Trigger		((1<<4)|2)	//��������
#define	DJI_Action_ShootSingal	((1<<4)|1)	//����
#define	DJI_Action_AUTOAIM		((2<<4)|2)	//����
#define	DJI_Action_Unknown2		((2<<4)|1)	//
#define	DJI_Action_Span				((2<<4)|3)	//С������ת

**************************************************/
float LookZangle;
uint8_t Lcount=0;
void DJI_Remote_Action(void)
{
	uint8_t i;
	static uint8_t Span_flag=0,Singal_flag=0,LookFlag=0;
	float temp_Zrpm=0,temp_Frpm=FRICRPM_RESET,temp_Trpm=0,temp_Yawrpm=0,temp_Trad=0;
	static float Wzrad[4] = {0.0},SavWz = 0,Zangle = 0;
	uint16_t Temp_pluse=1000;
	AIMMODE = REMOTEAIM;
	CHASSISMODE = NORMALCHASSIS;
	switch(DJI_Action_Flag)
	{
		case DJI_Action_Reset:;
		{
			Motor_Gimbal_Trigger.Trigger_circle = 0;//�������ٶȿ���
			temp_Trpm = 0;
			temp_Frpm = FRICRPM_RESET;
			LED_Control(0x01);
			LASER_OFF;
		}
		break;
		case DJI_Action_Fric:
		{
			temp_Frpm = FRICRPM_LOW;
			LED_Control(0x02);
		}
		break;
		case DJI_Action_Trigger:
		{
			temp_Frpm = FRICRPM_LOW;
			temp_Trpm = TRIGGER_SHOOTSPEED;
			Motor_Gimbal_Trigger.Trigger_circle = 0;//�������ٶȿ���;
			LED_Control(0x04);
		}
		break;
		case DJI_Action_AUTOAIM:
		{
			AIMMODE=AUTOAIM;
			LED_Control(0x08);
		}
		break;
		case DJI_Action_Unknown2:
		{
			Temp_pluse=1500;
			LED_Control(0x03);
		}
		break;
		case DJI_Action_Span:
		{
			LED_Control(0x05);
			CHASSISMODE=SPANCHASSIS;
			if (Span_flag == 0)
			{
				Zangle = IMUAngle.imuYaw;
				Span_flag = 1;
				LookZangle=Zangle;
			}
			temp_Yawrpm = 0;
			temp_Yawrpm = (rc.sw - DJI_CHMID)*MAXSPEEDY/DJI_MID;
			for (i=3;i>0;i--)
			{Wzrad[i] = Wzrad[i-1];}
			Wzrad[0] = IMUGyro.Wz;
			SavWz = (Wzrad[0]+Wzrad[1]+Wzrad[2]+Wzrad[3])/4.0f;
			temp_Yawrpm =temp_Yawrpm-SavWz*60/360;
			Span_Move_Calculate(Zangle);
			//temp_Zrpm=ZRPM_SPAN;
			//LookA = temp_Yawrpm;
			LASER_ON;
		}
		break;
		case DJI_Action_ShootSingal:
		{
			temp_Frpm = FRICRPM_LOW;
			if (Singal_flag==0)
			{
				Lcount++;
				Remote_Gimbal_Trigger.Trigger_rad += TRIGGER_RADSTEP;
				if (temp_Trad>=360)
					Remote_Gimbal_Trigger.Trigger_rad = Remote_Gimbal_Trigger.Trigger_rad-360.0f;
				Singal_Shoot_Calculate();
				Singal_flag=1;
			}
			LED_Control(0x09);
			LASER_OFF;
		}
		break;
		case DJI_Action_Brake:
		{
			LED_Control(0x06);
	//		Urgency_Brake();
		}
		break;
		default:	break;					
	}
		LookFlag = DJI_Action_Flag;
	if (DJI_Action_Flag!=DJI_Action_Span)
	{Span_flag = 0;}
	if (DJI_Action_Flag!=DJI_Action_ShootSingal)
	{Singal_flag = 0;}
	Remote_Gimbal_Trigger.Fric_rpm 		= temp_Frpm;
	Remote_Gimbal_Trigger.Trigger_rpm = temp_Trpm;
	Remote_Chassis_Rpm.Zrpm		+= temp_Zrpm;
	Remote_Gimbal_Yaw.Yaw_rpm += temp_Yawrpm;
	Helm_pluse = Temp_pluse;
}

int16_t PC_temp_rpm_up(int16_t PC_temp_rpm,uint8_t XYZFlag)
{
	int16_t temp_rpm_up = PC_temp_rpm;
/*******************�����ٶȲ�ͬ����***********************/
//	if (temp_rpm_up > 0)
//	{
//		temp_rpm_up = temp_rpm_up + RPM_STEP;
//		if (temp_rpm_up > temp_rpm_max)
//			temp_rpm_up = temp_rpm_max;
//	}
//	else
//	{
//		temp_rpm_up = temp_rpm_up - RPM_STEP;
//		if (temp_rpm_up < temp_rpm_min)
//			temp_rpm_up = temp_rpm_min;
//	} 
/******************�����ٶ���ͬ����***********************/
	if (XYZFlag)
	{
		temp_rpm_up = temp_rpm_up + RPM_STEP;
		if (temp_rpm_up > XYRPM_MAX)
			temp_rpm_up = XYRPM_MAX;
	}
	else
	{
		temp_rpm_up = temp_rpm_up + RPM_STEP;
		if (temp_rpm_up > ZRPM_MAX)
			temp_rpm_up = ZRPM_MAX;
	}
	return (temp_rpm_up);
}
int16_t PC_temp_rpm_down(int16_t PC_temp_rpm,uint8_t XYZFlag)
{
	int16_t temp_rpm_down = PC_temp_rpm;
	if (XYZFlag)
	{
		temp_rpm_down = temp_rpm_down - RPM_STEP;
		if (temp_rpm_down < 0)
			temp_rpm_down = 0;
	}
	else
	{
		temp_rpm_down = temp_rpm_down - RPM_STEP;
		if (temp_rpm_down < 0)
			temp_rpm_down = 0;
	}
	return (temp_rpm_down);
}
/*********************************************************************
PC���̿��ƴ���
���ܣ������̻ش�����ֵת��Ϊ���̵��˶�
���ã����̿��ƴ��롢PC��������
��ʽ���ٶȿ���
������� ����Ϊ�����˶�
		W
A		S		D
ʵ��ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
Shift Ϊ���ٰ�ť �����ά�˶�
Shift +			 	W
					A		S		D
ʵ�ֶ�Ӧ����ļ���ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
Ctrl Ϊ���ٰ�ť �����ά�˶�
Ctrl +			 	W
					A		S		D
ʵ�ֶ�Ӧ����ļ���ǰ�����ˡ�WA��ת��WD��ת //����SD��ת��AS��ת����
E��						�ٶ�ֵ��ʼ��
Shift	+	E��		ɲ����ͣ
Shift + Ctrl	��̨�Ƕȸ�λ
*********************************************************************/
void PC_Remote_Chassis(void)
{
	static int16_t PC_temp_rpm_X=0,PC_temp_rpm_Y=0,PC_temp_rpm_Z=0;//��������м��� ��Ϊ��ֵ
	int16_t PC_set_rpm_X=0,PC_set_rpm_Y=0,PC_set_rpm_Z=0;		//����ṹ���������
	switch (PC_KEY_Flag)
	{
		case PC_Motion_XP:
		{PC_set_rpm_X = XRPM_RESET;
//			if (PC_Press_Shift)
//				PC_set_rpm_X = PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag);
//			else if (PC_Press_Ctrl)
//				PC_set_rpm_X = PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag);
//			else
//				PC_set_rpm_X = PC_temp_rpm_X;
			PC_set_rpm_X = XRPM_RESET;
		}break;
		case PC_Motion_XN:
		{
//			if (PC_Press_Shift)
//				PC_set_rpm_X = (-PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag));
//			else if (PC_Press_Ctrl)
//				PC_set_rpm_X = (-PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag));
//			else
//				PC_set_rpm_X = (-PC_temp_rpm_X);
			PC_set_rpm_X = -XRPM_RESET;
		}break;
		case PC_Motion_YP:
		{
//			if (PC_Press_Shift)
//				PC_set_rpm_Y = PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag);
//			else if (PC_Press_Ctrl)
//				PC_set_rpm_Y = PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag);
//			else
//				PC_set_rpm_Y = PC_temp_rpm_Y;
			PC_set_rpm_Y = YRPM_RESET;
		}break;
		case PC_Motion_YN:
		{
//			if (PC_Press_Shift)
//				PC_set_rpm_Y = (-PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag));
//			else if (PC_Press_Ctrl)
//				PC_set_rpm_Y = (-PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag));
//			else
//				PC_set_rpm_Y = (-PC_temp_rpm_Y);
			PC_set_rpm_Y = -YRPM_RESET;
		}break;
		case PC_Motion_ZP:
		{
//			if (PC_Press_Shift)
//				PC_set_rpm_Z = PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag);
//			else if (PC_Press_Ctrl)
//				PC_set_rpm_Z = PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag);
//			else
//				PC_set_rpm_Z = PC_temp_rpm_Z;
			PC_set_rpm_Z = ZRPM_RESET;
		}break;
		case PC_Motion_ZN:
		{
//			if (PC_Press_Shift)
//				PC_set_rpm_Z = (-PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag));
//			else if (PC_Press_Ctrl)
//				PC_set_rpm_Z = (-PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag));
//			else
//				PC_set_rpm_Z = (-PC_temp_rpm_Z);
			PC_set_rpm_Z = -ZRPM_RESET;
		}break;
		case PC_Motion_WS:
		{
			PC_temp_rpm_X = PC_temp_rpm_X + RPM_STEP;
			PC_set_rpm_X += PC_temp_rpm_X;
		}break;
		case PC_Motion_SS:
		{
			PC_temp_rpm_X = PC_temp_rpm_X + RPM_STEP;
			PC_set_rpm_X += PC_temp_rpm_X;
			PC_set_rpm_X = -PC_set_rpm_X;
		}break;
		case PC_Motion_ES:
		{
			PC_temp_rpm_Z = PC_temp_rpm_Z + RPM_STEP;
			PC_set_rpm_Z += PC_temp_rpm_Z;
		}break;
		case PC_Motion_QS:
		{
			PC_temp_rpm_Z = PC_temp_rpm_Z + RPM_STEP;
			PC_set_rpm_Z += PC_temp_rpm_Z;
			PC_set_rpm_Z = -PC_set_rpm_Z;
		}break;
		default:
		{
			PC_set_rpm_X = 0;
			PC_set_rpm_Y = 0;
			PC_set_rpm_Z = 0;
			PC_temp_rpm_X=0;
			PC_temp_rpm_Y=0;
			PC_temp_rpm_Z=0;
		}break;//���̾�ֹ�������ٶ�Ϊ0
	}
//	if (PC_Press_W)
//		PC_set_rpm_X = XRPM_RESET;
//	if (PC_Press_S)
//		PC_set_rpm_X = -XRPM_RESET;
//	if (PC_Press_D)
//		PC_set_rpm_Y = YRPM_RESET;
//	if (PC_Press_A)
//		PC_set_rpm_Y = -YRPM_RESET;
//	if (PC_Press_E)
//		PC_set_rpm_Z = ZRPM_RESET;
//	if (PC_Press_Q)
//		PC_set_rpm_Z = -ZRPM_RESET;
//	if (PC_Press_W&&PC_Press_Shift)
//	{
//		PC_temp_rpm_X++;
//		PC_set_rpm_X += PC_temp_rpm_X;
//	}
//	if (PC_Press_S&&PC_Press_Shift)
//	{
//		PC_temp_rpm_X++;
//		PC_set_rpm_X += PC_temp_rpm_X;
//		PC_set_rpm_X = -PC_set_rpm_X;
//	}
	
	/*****************���ٶ�ֵ����������API***********************/
	Remote_Chassis_Rpm.Xrpm = PC_set_rpm_X;
	Remote_Chassis_Rpm.Yrpm = PC_set_rpm_Y;
	Remote_Chassis_Rpm.Zrpm = PC_set_rpm_Z;
}
/******************************************************************
�����̨����
���ܣ������ƫ�Ƶ��ٶ�ת��Ϊ��̨yaw��pitch��ת���ٶ�
���ã���̨���ƴ��롢PC��������
��ʽ���ٶȿ���
������� ����Ϊ��̨�˶�
mouse X ��̨��yaw����ת��		//����λ 360��
mouse Y ��̨��pitch����ת��	//����λ ������
mouse Z ����

*******************************************************************/
void PC_Remote_Gimball(void)
{
	float temp_Yspeed=0,temp_Pspeed=0;
	static float temp_Yangle=YAW_RESETRAD,temp_Pangle=PITCH_RESETRAD;
	temp_Yangle += (PC_Mouse_X/PC_YawVAL);
	if (temp_Yangle>YAW_MAXRAD)
	{temp_Yangle=YAW_MAXRAD;}
	if (temp_Yangle<YAW_MINRAD)
	{temp_Yangle=YAW_MINRAD;}
	temp_Pangle += (PC_Mouse_Y/PC_PitchVAL);
	if (temp_Pangle>PITCH_MAXRAD)
	{temp_Pangle=PITCH_MAXRAD;}
	if (temp_Pangle<PITCH_MINRAD)
	{temp_Pangle=PITCH_MINRAD;}
	temp_Yspeed = PC_Mouse_X/PC_YawVAL;
	temp_Pspeed = PC_Mouse_Y/PC_PitchVAL;
	if (PC_Press_Ctrl)
	{
		if (PC_Press_Shift)
		{
//		if (Remote_Gimbal_Yaw.Yaw_rad>0)
		temp_Yangle = YAW_RESETRAD;
		temp_Pangle = PITCH_RESETRAD;
		temp_Yspeed = 0;
		temp_Pspeed = 0;
		}
	}
	Remote_Gimbal_Yaw.Yaw_rad = -temp_Yangle;
	Remote_Gimbal_Yaw.Yaw_rpm = -temp_Yspeed;
	Remote_Gimbal_Pitch.Pitch_rad = -temp_Pangle;
	Remote_Gimbal_Pitch.Pitch_rpm = -temp_Pspeed;
}

/******************************************************************
����������
���ܣ�����갴����ֵת��Ϊ��̨����ָ��
���ã���̨���ƴ��롢PC��������
��ʽ���߼�����
������� ����Ϊ��̨����
mouse left_key 	�������
mouse right_key	�Ҽ�����
shift ����/�ر� Ħ����
*******************************************************************/
void PC_Remote_Action(void)
{
	static uint8_t Span_flag=0,Singal_flag=0,Shoot_flag=0,Fric_flag=0;
	float temp_Zrpm=0,temp_Frpm=FRICRPM_RESET,temp_Trpm=0,temp_Yawrpm=0,temp_Trad=0;
	AIMMODE = REMOTEAIM;
	CHASSISMODE = NORMALCHASSIS;
	if (PC_Press_Ctrl)
	{
		if (PC_Press_S)
		{
			if (Shoot_flag==0)
			{
				Shoot_flag=1;
				Fric_flag++;
				if (Fric_flag==100)
					Fric_flag=0;
			}
		}
	}
	if (Fric_flag%2)
		temp_Frpm =1700;
	else
		temp_Frpm = FRICRPM_RESET;
	if (PC_Press_Ctrl==0&&PC_Press_S==0)
	{Shoot_flag=0;}
	if ((PC_Mouse_Left) && (~PC_Mouse_Right))
	{
			temp_Frpm = FRICRPM_LOW;
			if (Singal_flag==0)
			{
				Lcount++;
				Remote_Gimbal_Trigger.Trigger_rad += TRIGGER_RADSTEP;
				if (temp_Trad>=360)
					Remote_Gimbal_Trigger.Trigger_rad = Remote_Gimbal_Trigger.Trigger_rad-360.0f;
				Singal_Shoot_Calculate();
				Singal_flag=1;
			}
	}//��������
	else if ((PC_Mouse_Right) && (~PC_Mouse_Left))
	{
		temp_Frpm = FRICRPM_LOW;
		temp_Trpm = TRIGGER_SHOOTSPEED;
		Singal_flag = 0;
		Motor_Gimbal_Trigger.Trigger_circle = 0;//�������ٶȿ���;
	}//����
	else
	{
		Singal_flag = 0;
	}//�޶���
	Remote_Gimbal_Trigger.Fric_rpm 		= temp_Frpm;
	Remote_Gimbal_Trigger.Trigger_rpm = temp_Trpm;
}

