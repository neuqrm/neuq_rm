

#include "remote.h"

Chassis_Rpm_stc			Remote_Chassis_Rpm	=	{0};
Gimbal_Yaw_stc			Remote_Gimbal_Yaw		=	{0};
Gimbal_Pitch_stc		Remote_Gimbal_Pitch	=	{0};
Gimbal_Trigger_stc	Remote_Gimbal_Trigger	=	{0};


/**************************************************
战车整体DJI遥控器控制
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
战车整体PC遥控器控制
**************************************************/
void PC_Remote_Control(void)
{
		PC_Remote_Chassis();
		PC_Remote_Gimball();
		PC_Remote_Action();
}
/***************************************************
遥控器底盘控制代码
功能：将摇杆回传的数值转化为底盘的运动
调用：底盘控制代码、遥控器接收数据
方式：速度控制
输出：无 表现为底盘运动
右手平面运动
左右旋转运动
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
遥控器云台控制代码
功能：将摇杆和拨轮回传的数值转化为云台的运动
调用：云台控制代码、遥控器接收数据
方式：角度控制 or 速度控制
输出：无 表现为云台运动
左手pitch运动
拨轮yaw运动
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
战车动作控制
功能：接收推杆数据，判断战车动作
调用：云台、底盘控制代码，遥控器接收数据
方式：逻辑控制
输出：无 表现为战车动作
通道4~5	S1~2 左S1 右S2
上	=	1；		中	=	3；		下	=	2；
#define DJI_Action_Flag		((rc.s1<<4)|rc.s2)	//推杆选择云台动作
#define	DJI_Action_Reset			((3<<4)|3)	//复位状态 无动作
#define	DJI_Action_Change			((3<<4)|1)	//遥控器和键盘切换
#define	DJI_Action_Brake			((3<<4)|2)		//紧急刹车
#define	DJI_Action_Fric				((1<<4)|3)	//开摩擦轮
#define	DJI_Action_Trigger		((1<<4)|2)	//开播弹轮
#define	DJI_Action_ShootSingal	((1<<4)|1)	//点射
#define	DJI_Action_AUTOAIM		((2<<4)|2)	//自瞄
#define	DJI_Action_Unknown2		((2<<4)|1)	//
#define	DJI_Action_Span				((2<<4)|3)	//小陀螺旋转

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
			Motor_Gimbal_Trigger.Trigger_circle = 0;//拨弹轮速度控制
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
			Motor_Gimbal_Trigger.Trigger_circle = 0;//拨弹轮速度控制;
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
/*******************方向速度不同代码***********************/
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
/******************方向速度相同代码***********************/
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
PC底盘控制代码
功能：将键盘回传的数值转化为底盘的运动
调用：底盘控制代码、PC接收数据
方式：速度控制
输出：无 表现为底盘运动
		W
A		S		D
实现前进后退、WA左转、WD右转 //其中SD左转、AS右转待定
Shift 为加速按钮 配合三维运动
Shift +			 	W
					A		S		D
实现对应方向的加速前进后退、WA左转、WD右转 //其中SD左转、AS右转待定
Ctrl 为加速按钮 配合三维运动
Ctrl +			 	W
					A		S		D
实现对应方向的减速前进后退、WA左转、WD右转 //其中SD左转、AS右转待定
E键						速度值初始化
Shift	+	E键		刹车急停
Shift + Ctrl	云台角度复位
*********************************************************************/
void PC_Remote_Chassis(void)
{
	static int16_t PC_temp_rpm_X=0,PC_temp_rpm_Y=0,PC_temp_rpm_Z=0;//计算过程中间量 恒为正值
	int16_t PC_set_rpm_X=0,PC_set_rpm_Y=0,PC_set_rpm_Z=0;		//结算结构电机控制量
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
		}break;//底盘静止，控制速度为0
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
	
	/*****************将速度值赋予电机控制API***********************/
	Remote_Chassis_Rpm.Xrpm = PC_set_rpm_X;
	Remote_Chassis_Rpm.Yrpm = PC_set_rpm_Y;
	Remote_Chassis_Rpm.Zrpm = PC_set_rpm_Z;
}
/******************************************************************
鼠标云台控制
功能：将鼠标偏移的速度转化为云台yaw和pitch的转动速度
调用：云台控制代码、PC接收数据
方式：速度控制
输出：无 表现为云台运动
mouse X 云台的yaw方向转动		//无限位 360°
mouse Y 云台的pitch方向转动	//有限位 俯仰角
mouse Z 暂无

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
鼠标射击控制
功能：将鼠标按键的值转化为云台动作指令
调用：云台控制代码、PC接收数据
方式：逻辑控制
输出：无 表现为云台动作
mouse left_key 	左键连发
mouse right_key	右键点射
shift 开启/关闭 摩擦轮
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
	}//开火连射
	else if ((PC_Mouse_Right) && (~PC_Mouse_Left))
	{
		temp_Frpm = FRICRPM_LOW;
		temp_Trpm = TRIGGER_SHOOTSPEED;
		Singal_flag = 0;
		Motor_Gimbal_Trigger.Trigger_circle = 0;//拨弹轮速度控制;
	}//点射
	else
	{
		Singal_flag = 0;
	}//无动作
	Remote_Gimbal_Trigger.Fric_rpm 		= temp_Frpm;
	Remote_Gimbal_Trigger.Trigger_rpm = temp_Trpm;
}

