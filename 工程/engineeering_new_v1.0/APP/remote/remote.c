#include "remote.h"
#include "motor.h"
#include "bsp_led.h"


/**************************************************
战车整体DJI遥控器控制
**************************************************/
void DJI_Remote_Control(void)
{
		DJI_Remote_Chassis();
		DJI_Remote_Gimball();
		DJI_Remote_Action();
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
//	float tempX=0,tempY=0,tempZ=0;
//	tempX = (rc.ch1-DJI_CHMID)/DJI_XVAL;
//	tempY = (rc.ch0-DJI_CHMID)/DJI_YVAL;
//	tempZ = (rc.ch2-DJI_CHMID)/DJI_ZVAL;
//	Chassis_Ctrl(ChassisX,tempX);
//	Chassis_Ctrl(ChassisY,tempY);
//	Chassis_Ctrl(ChassisZ,tempZ);
}
/**************************************************
遥控器云台控制代码
功能：将摇杆和拨轮回传的数值转化为云台的运动
调用：云台控制代码、遥控器接收数据
方式：角度控制
输出：无 表现为云台运动
左手pitch运动
拨轮yaw运动
**************************************************/
void DJI_Remote_Gimball(void)
{
//	uint16_t Yaw_angle,Pitch_angle;
//	Yaw_angle 	= (rc.sw - DJI_MIN)*(YAW_MAXANGLE - YAW_MINANGLE)/(DJI_MAX - DJI_MIN)+YAW_MINANGLE;
//	Pitch_angle = (rc.ch3 - DJI_MIN)*(PITCH_MAXANGLE - PITCH_MINANGLE)/(DJI_MAX - DJI_MIN)+PITCH_MINANGLE;
//	Gimbal_Ctrl(GimbalYaw,0,Yaw_angle);
//	Gimbal_Ctrl(GimbalPitch,0,Pitch_angle);	
}
/**************************************************
战车动作控制
功能：接收推杆数据，判断战车动作
调用：云台、底盘控制代码，遥控器接收数据
方式：逻辑控制
输出：无 表现为战车动作
	Action_Rest			((3<<8)|3)	//复位状态 无动作
	Action_Shoot_H	((1<<8)|2)	//高速开火	摩擦轮射速较大，拨弹轮高速转动
	Action_Fire_H		((1<<8)|1)	//高速点射	摩擦轮射速较大，拨弹轮角度转动
	Action_Shoot_L	((2<<8)|2)	//低速开火	摩擦轮射速较小，拨弹轮高速转动
	Action_Fire_L		((2<<8)|1)	//低速点射	摩擦轮射速较小，拨弹轮角度转动
	Action_Unknown1	((1<<8)|3)	//待定可以增加小陀螺等
	Action_Unknown2	((2<<8)|3)	//
	Action_Unknown3	((3<<8)|1)	//
	Action_Unknown4	((3<<8)|2)	//
**************************************************/
void DJI_Remote_Action(void)
{
	switch(DJI_Action_Flag)
	{
		case DJI_Action_Reset:;
		{
			LED_Control(0x11);
			LASER_OFF;
		}
		break;
		case DJI_Action_Shoot_H:
		{
			LED_Control(0x22);
		}
		break;
		case DJI_Action_Fire_H:
		{
			LED_Control(0x44);
		}
		break;
		case DJI_Action_Shoot_L:
		{
			LED_Control(0x88);
		}
		break;
		case DJI_Action_Fire_L:
		{
			LED_Control(0x81);
		}
		break;
		/*case DJI_Action_Unknown1:
		{
			LED_Control(0x42);
			pid_clear(&pid_wheel_speed[0]);
			pid_clear(&pid_wheel_speed[1]);
			pid_clear(&pid_wheel_speed[2]);
			pid_clear(&pid_wheel_speed[3]);
			LASER_ON;
		}
		break;*/
		case DJI_Action_Unknown2:
		{
			LED_Control(0x24);
			LASER_ON;
		}
		break;
		case DJI_Action_Unknown3:
		{
			LED_Control(0x18);
		}
		break;
		case DJI_Action_Unknown4:
		{
			LED_Control(0x33);
		}
		break;
		default:	break;					
		
	}
	
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
	
	int16_t PC_temp_rpm_X,PC_temp_rpm_Y,PC_temp_rpm_Z;//计算过程中间量 恒为正值
	int16_t PC_set_rpm_X,PC_set_rpm_Y,PC_set_rpm_Z;		//结算结构电机控制量
	switch (PC_KEY_VALUE & 0x0004)
	{
		case PC_Motion_XP:
		{
			if (PC_Press_Shift)
				PC_set_rpm_X = PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag);
			else
				PC_set_rpm_X = PC_temp_rpm_X;
		}break;
		case PC_Motion_XN:
		{
			if (PC_Press_Shift)
				PC_set_rpm_X = (-PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = (-PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag));
			else
				PC_set_rpm_X = (-PC_temp_rpm_X);
		}break;
		case PC_Motion_YP:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Y = PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag);
			else
				PC_set_rpm_Y = PC_temp_rpm_Y;
		}break;
		case PC_Motion_YN:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Y = (-PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = (-PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag));
			else
				PC_set_rpm_Y = (-PC_temp_rpm_Y);
		}break;
		case PC_Motion_ZP:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Z = PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag);
			else
				PC_set_rpm_Z = PC_temp_rpm_Z;
		}break;
		case PC_Motion_ZN:
		{
			if (PC_Press_Shift)
				PC_set_rpm_Z = (-PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = (-PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag));
			else
				PC_set_rpm_Z = (-PC_temp_rpm_Z);
		}break;
		case PC_Motion_Reset:
		{
			PC_temp_rpm_X = XRPM_RESET;
			PC_temp_rpm_Y = YRPM_RESET;
			PC_temp_rpm_Z = ZRPM_RESET;
		}break;//底盘速度计算量重置
		case PC_Action_Reset:
		{
			
		}break;//云台角度回正
		default:
		{
			PC_set_rpm_X = 0;
			PC_set_rpm_Y = 0;
			PC_set_rpm_Z = 0;
		}break;//底盘静止，控制速度为0
	}
	//将速度值赋予电机控制API

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
	
}

/******************************************************************
鼠标云台控制
功能：将鼠标按键的值转化为云台动作指令
调用：云台控制代码、PC接收数据
方式：逻辑控制
输出：无 表现为云台动作
mouse left_key 	左键连发
mouse right_key	右键点射
*******************************************************************/
void PC_Remote_Action(void)
{
	if (PC_Mouse_Left & ~PC_Mouse_Right)
	{
		
	}//开火连射
	else if (PC_Mouse_Right & PC_Mouse_Left)
	{
		
	}//点射
	else
	{
		
	}//无动作
}



