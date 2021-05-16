#include "remote.h"
#include "motor.h"
#include "bsp_led.h"
#include "remote_code.h"
#include "bsp_dbus.h"
#include "kinematic.h"
#include "fric.h"
#include "stm32f4xx_tim.h"
#include "gimbal.h"
#include "imuReader.h"              
#include <math.h>
#include "mode.h"
#include "delay.h"
#include "My_Init.h"

//调用remote_code.c里的中间变量和计算函数
extern float x_speed,y_speed,r_speed,trigger_speed,theta,trigger_angle,yaw_angle1,pitch_angle1;
extern float cx_speed,cy_speed,fric_angular,yaw_angular,pitch_angular;
extern int flag;
extern int trigger_flag;
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);
void set_trigger_control(void);

int CONTROL_MODE=0; //启动电脑控制 0为遥控器控制 1为电脑控制

void control_remote_mode_choose(void)
{
	x_speed=0;
	y_speed=0;
	r_speed=0;
	if(CONTROL_MODE==0)
	DJI_Remote_Control();
	else PC_Remote_Control();
	
}
/**************************************************
战车整体DJI遥控器控制
**************************************************/
void DJI_Remote_Control(void)
{
		DJI_Remote_Chassis();
		DJI_Remote_Gimball();
		DJI_Remote_Action();
	    if((control_mode) == DJi_Remote_Control)
		{
			y_speed = y_speed;
			r_speed = -r_speed; //取反，使逆时针旋转为正向
		}
		dji_remote_assignment();  //将上述遥控计算数据赋值，执行部分在robomove里
        set_trigger_control();
}

/**************************************************
战车整体PC遥控器控制
**************************************************/
void PC_Remote_Control(void)
{
		PC_Remote_Chassis();
		PC_Remote_Gimball();
		PC_Remote_Action();
	    fric_angular=1000;
	    dji_remote_assignment();
	   
	    Kinematics.trigger.target_angle+=trigger_angle; 
}
/***************************************************
遥控器底盘控制代码`
功能：将摇杆回传的数值转化为底盘的运动
调用：底盘控制代码、遥控器接收数据
方式：速度控制
输出：无 表现为底盘运动
右手平面运动
左右旋转运动
***************************************************/

void DJI_Remote_Chassis(void)
{
   x_speed=caculate_linear_speed(DJI_Motion_X,y_initial_value,y_min_value,y_max_value);
   y_speed=caculate_linear_speed(DJI_Motion_Y,x_initial_value,x_min_value,x_max_value);
   r_speed=caculate_rotational_speed(DJI_Motion_Z,z_initial_value,z_min_value,z_max_value);  
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
//模式控制在主函数里的模式控制里选择
void DJI_Remote_Gimball(void)
{
	switch(gimbal_modes)
	{
				case(gimbal_pwm_mode):  				//pwm模式下控制云台转角

			  	pwm_pulse_p=caculate_gimbal_pitch_angle(DJI_Motion_Pitch,i_initial_value,i_min_value,i_max_value);
				  pwm_pulse_y=caculate_gimbal_yaw_angle(DJI_Motion_Yaw,x_initial_value,x_min_value,x_max_value);
				
				break;
				case(gimbal_can_mode):           //can模式下控制云台转角
					Kinematics.pitch.target_angle = caculate_gimbal_pitch_angle(DJI_Motion_Pitch,i_initial_value,i_min_value,i_max_value);
					Kinematics.yaw.target_angle = caculate_gimbal_yaw_angle(DJI_Motion_Yaw,x_initial_value,x_min_value,x_max_value);
                break;
				default:break;
     }
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
		case DJI_Action_Reset:;//复位状态 摩擦轮波弹轮都是 空
		{
			LED_Control(0x11);
			LASER_OFF;
			
			trigger_angle = 0;
			fric_angular=1000;
		}
		break;
		case DJI_Action_Shoot_H:
		{
			LED_Control(0x22);
			
			trigger_angle = 0;
			fric_angular=1300;
			
		}
		break;
		case DJI_Action_Fire_H:
		{
			LED_Control(0x44);
			fric_angular=1300;
					
			if(trigger_angle!=1100&&trigger_angle!=1800) trigger_flag=0;  
			trigger_angle = 1100; 
		}
		break;
		case DJI_Action_Shoot_L:
		{
			LED_Control(0x88);
			fric_angular=1300;
					
			if(trigger_angle!=900&&trigger_angle!=1800) trigger_flag=0;  
			trigger_angle = 1800; 
		}
		break;
		case DJI_Action_Fire_L:
		{
			LED_Control(0x81);
			trigger_angle = 0;
			fric_angular=1300;
		}
		break;

		case DJI_Action_Unknown1:
		{
			LED_Control(0x24);
			LASER_ON;
			trigger_angle = 0;
			fric_angular=0;
		}
		break;
		case DJI_Action_Unknown3://启动电脑控制模式  3 1
		{
			LED_Control(0x18);
			CONTROL_MODE=1;
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
	int16_t PC_temp_rpm_X=0,PC_temp_rpm_Y=0,PC_temp_rpm_Z=0;//计算过程中间量 恒为正值
	int16_t PC_set_rpm_X=0,PC_set_rpm_Y=0,PC_set_rpm_Z=0;		//结算结构电机控制量
	switch (PC_KEY_Flag & 0x0004)
	{
		case PC_Motion_XP://前进
		{
			PC_temp_rpm_X=XRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_X = PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag);
			else
				PC_set_rpm_X = PC_temp_rpm_X;
		}break;
		case PC_Motion_XN://后退
		{
			PC_temp_rpm_X=XRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_X = (-PC_temp_rpm_up(PC_temp_rpm_X,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_X = (-PC_temp_rpm_down(PC_temp_rpm_X,XY_Flag));
			else
				PC_set_rpm_X = (-PC_temp_rpm_X);
		}break;
		case PC_Motion_YP://向左平移
		{
			PC_temp_rpm_Y=YRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Y = PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag);
			else
				PC_set_rpm_Y = PC_temp_rpm_Y;
		}break;
		case PC_Motion_YN://向右平移
		{
			PC_temp_rpm_Y=YRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Y = (-PC_temp_rpm_up(PC_temp_rpm_Y,XY_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Y = (-PC_temp_rpm_down(PC_temp_rpm_Y,XY_Flag));
			else
				PC_set_rpm_Y = (-PC_temp_rpm_Y);
		}break;
		case PC_Motion_ZP://向左转
		{
			PC_temp_rpm_Z=ZRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Z = PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag);
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag);
			else
				PC_set_rpm_Z = PC_temp_rpm_Z;
		}break;
		case PC_Motion_ZN://向右转
		{
			PC_temp_rpm_Z=ZRPM_RESET;
			if (PC_Press_Shift)
				PC_set_rpm_Z = (-PC_temp_rpm_up(PC_temp_rpm_Z,Z_Flag));
			else if (PC_Press_Ctrl)
				PC_set_rpm_Z = (-PC_temp_rpm_down(PC_temp_rpm_Z,Z_Flag));
			else
				PC_set_rpm_Z = (-PC_temp_rpm_Z);
		}break;
		case PC_Motion_Reset:
		{
			PC_temp_rpm_X = 0;
			PC_temp_rpm_Y = 0;
			PC_temp_rpm_Z = 0;
		}break;//底盘速度计算量重置
		case PC_Action_Reset:
		{
			motor_Init_angle(); 
		}break;//云台角度回正
		default:
		{
			/*
			if(flag==0)
			{
				fric_angular=1200;
				flag=1;
			}
			else 
			{
				fric_angular=1000;
				flag=0;
			}
			*/
		}break;//启动波弹轮
	}
	/*****************将速度值赋予电机控制API***********************/
	x_speed=PC_set_rpm_X;
	y_speed=PC_set_rpm_Y;
	r_speed=PC_set_rpm_Z;
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
mouse left_key 	左键点射
mouse right_key	右键连发
*******************************************************************/
void PC_Remote_Gimball(void)
{
	if(PC_Mouse_Left) //点射
	{
		trigger_angle = 900;
		fric_angular=1600;   //1300
	}
	if(PC_Mouse_Right) //连射
	{
		trigger_angle = 2700;
		fric_angular=1600;   //1300
	}
	if(PC_Mouse_X||PC_Mouse_Y)
	{
		gimbal_loop=speed_loop;
		yaw_angular=PC_Mouse_X*40/32767;
		pitch_angular=PC_Mouse_Y*20/32767;
	}	
	else gimbal_loop=position_loop;
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


/**
  * @brief  将遥控器摇杆输出映射到机器人三轴速度上
  * @param  width：通道值 
  *         mid：通道中间值 
  *         min：通道输出最小值
  *         max：通道输出最大值
  */
static float caculate_linear_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//中间消除波动
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_linear_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_linear_speed);
  else
    speed=0;
  return speed;                
}

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//中间消除波动
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_rotational_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_rotational_speed);
  else
    speed=0;
  return speed;                
}

static float caculate_gimbal_pitch_angle(int width,int mid,int min,int max)
{
	float pwm_can;                         //此变量返回计算得出的pwm脉宽或者can模式下的机械角度值
	switch(gimbal_modes)
	{
		case(gimbal_pwm_mode):
			
	   pwm_can=BASIC_PITCH_ANGLE_PWM;
		 if(width>=(mid+2))
		  pwm_can=(BASIC_PITCH_ANGLE_PWM - 1.0*(width-(mid+2))/(max-(mid+2))*210);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_PITCH_ANGLE_PWM + 1.0*((mid-2)-width)/((mid-2)-min)*105);
	   else
		  pwm_can=BASIC_PITCH_ANGLE_PWM;
	
		 break;
		 
		case(gimbal_can_mode):
		 pwm_can=BASIC_PITCH_ANGLE_CAN;
		 if(width>=(mid+2))
		  pwm_can=(BASIC_PITCH_ANGLE_CAN - 1.0*(width-(mid+2))/(max-(mid+2))*2047);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_PITCH_ANGLE_CAN + 1.0*((mid-2)-width)/((mid-2)-min)*1023);
	   else
		  pwm_can=BASIC_PITCH_ANGLE_CAN;		
		 
		 break;
		 default:break;
   }
	return pwm_can;
}

static float caculate_gimbal_yaw_angle(int width,int mid,int min,int max)
{
	float pwm_can;
	switch(gimbal_modes)
	{
		case(gimbal_pwm_mode):
	    pwm_can=BASIC_YAW_ANGLE_PWM;
		 if(width>=(mid+2))
	 	  pwm_can=(BASIC_YAW_ANGLE_PWM - 1.0*(width-(mid+2))/(max-(mid+2))*420);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_YAW_ANGLE_PWM + 1.0*((mid-2)-width)/((mid-2)-min)*420);
	   else
		  pwm_can=BASIC_YAW_ANGLE_PWM;

		 break;
		 
		 case(gimbal_can_mode):
		  pwm_can=BASIC_YAW_ANGLE_CAN;
		 if(width>=(mid+2))
		  pwm_can=(BASIC_YAW_ANGLE_CAN - 1.0*(width-(mid+2))/(max-(mid+2))*4095);
	   else if(width<=(mid-2))
	    pwm_can=(BASIC_YAW_ANGLE_CAN + 1.0*((mid-2)-width)/((mid-2)-min)*4095);
	   else
		  pwm_can=BASIC_YAW_ANGLE_CAN;
		 
		 break;
		 default:break;
		 
   }
		 return pwm_can;
}




