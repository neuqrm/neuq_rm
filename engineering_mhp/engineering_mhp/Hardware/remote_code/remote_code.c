/**
  ******************************************************************************
  * @file    Project/HARDWARE/remote_code.c 
  * @author  Siyuan Qiao & Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention 遥控器控制程序
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/

#include "remote_code.h"
#include "DJi_remote.h"
#include "motor.h"
#include "kinematic.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "delay.h"

u8 Control_Mode = control_mode;

//内部全局变量，方便调试
float x_speed=0,y_speed=0,r_speed=0,trigger_speed=0,theta=0;
float cx_speed=0,cy_speed=0;
int flag = 0;
//内部函数声明
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);

/**
  * @brief  遥控代码，将遥控器输出对应到机器人具体动作上，放在定时器里不断地刷
  */
void Remote_Control()    //这个函数里就不断地判断每个通道的值，如果满足条件就做相应动作
{	
	if(Remote_control_mode>remote_min_value && Remote_control_mode<remote_max_value)		//如果满足遥控条件
	{
		//标志位改为遥控模式
		Control_Mode &= remote_control;											//修改Control_Mode第二位为0
	}
	else
	{
		//标志位改为自动模式
		Control_Mode |= auto_control;												//修改Control_Mode第二位为1		
	}
	
	if((Control_Mode & auto_control) != auto_control)			//如果控制模式不等于自动控制，即遥控控制
	{
		if(Remote_control_mode == chassis_CH_width)
		{  
			x_speed=caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
			y_speed=caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			r_speed=caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value,r_max_value);  
		}						
	}
			
		if((Control_Mode&DJi_Remote_Control) == DJi_Remote_Control)
		{
			y_speed = y_speed;
			r_speed = -r_speed; //取反，使逆时针旋转为正向
		}
		else if((Control_Mode&FS_Remote_Control) == FS_Remote_Control)		//因为FS_Remote_Control = 0，因此判断时必须放在else if里
		{
			y_speed = -y_speed;
		}
		Kinematics.target_velocities.linear_x=x_speed;//放在robomove中执行.
		Kinematics.target_velocities.linear_y=y_speed;
		Kinematics.target_velocities.angular_z=r_speed;
		trigger_control(trigger_speed);	
	}
	



// 函数: caculate_speed()
// 描述: 将遥控器摇杆输出映射到机器人三轴速度上
// 参数：width：通道值 
//			 mid：通道中间值 
//			 min：通道输出最小值
//       max：通道输出最大值
// 输出：对应的速度值
//内部函数，用户无需调用
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





