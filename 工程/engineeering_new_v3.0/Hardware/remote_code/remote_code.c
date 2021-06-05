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

#include "kinematic.h"
#include "remote_code.h"
#include "motor.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "delay.h"
#include "flag.h"
#include "stepper_motor.h"
#include "bsp_dbus.h"
//内部函数声明
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);

extern int distance;

int a =0;
uint16_t pulse_cnt = 0;   //步进电机脉冲计数
uint16_t i = 0;           //抓取矿石流程步骤层
/**
  * @brief  遥控代码，将遥控器输出对应到机器人具体动作上，放在定时器里不断地刷
  */
void Remote_Control()    //这个函数里就不断地判断每个通道的值，如果满足条件就做相应动作
{	
	
	if(!LEFT_LEVER==0)			
	{
		
		if(LEFT_LEVER == keyboard_mode)  //键盘控制模式
		{
					if(RIGHT_LEVER==3)//右拨杆中		
					{   
						stepper_motor_off();
						TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE );
					}						
					if(RIGHT_LEVER==1)//右拨杆上
					{
						a=0;
						TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );	
						stepper_motor_up();
					}
					if(RIGHT_LEVER==2)//右拨杆下
					{
						a=1;
						TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );	
						stepper_motor_down();
					}

			if(key_board&ws_key)
		}
		
		if(LEFT_LEVER == catch_mineral_up | LEFT_LEVER == catch_mineral_down)  //左拨杆中或者右
		{  
			Liner_X   = caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
			Liner_Y   = caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			Angular_Z = caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value, r_max_value); 
			
			if(LEFT_LEVER == catch_mineral_up)  //左拨杆中
			{				
					if(i<=4)
					{
						if(RIGHT_LEVER==3 && lever_flag==0)
							lever_flag=1;
						if(RIGHT_LEVER==1 && lever_flag==1)
						{
							i++;
							lever_flag=0;
						}
					}
					if(i==1)
						Solenoid_valve_flag=1;
					if(i==2)
						Solenoid_handle_flag=1;
					if(i==3)
						distance=20;
					if(i==4)
					{ 
						Kinematics.handle_L.target_angle=180;
						Kinematics.handle_R.target_angle=180;
					}
					if(i==5)
					{
						Kinematics.handle_L.target_angle=-180;
						Kinematics.handle_R.target_angle=-180;
					}
				
			}
			if(LEFT_LEVER == catch_mineral_down) //左拨杆上
				{
					if(i>0)
					{
						if(RIGHT_LEVER==3 && lever_flag==0)
							lever_flag=1;
						if(RIGHT_LEVER==1 && lever_flag==1)
						{
							i--;
							lever_flag=0;
						}
					}
					if(i==1)
						Solenoid_valve_flag=0;
					if(i==2)
						Solenoid_handle_flag=0;
					if(i==3)
						distance=-20;
					if(i==4)
					{
						Kinematics.handle_R.target_angle=-180;
					}
					if(i==5)		
					{
						motor5.count=0;
						motor6.count=0;
						Kinematics.handle_L.target_angle=180;
						Kinematics.handle_R.target_angle=180;					
					}	
			}
		}
		if(RIGHT_LEVER == 2)    //右拨杆下
		{
		
		}
	}
	if(1)
	{
		  if((key_board&W_key) == ws_key)
			 {
				if((key_board&W_key) == W_key)
					Liner_X = max_base_linear_speed;
				if((key_board&S_key) == S_key)
					Liner_X = -max_base_linear_speed;
			 }
			else
				Liner_X = 0;
			if(key_board&ad_key)
			{
				if((key_board&A_key) == A_key)
					Angular_Z = -max_base_rotational_speed;
				if((key_board&D_key) == D_key)
					Angular_Z = max_base_rotational_speed;	
			}
			else
				Angular_Z = 0;
			if(key_board&qe_key)
				{
				if((key_board&Q_key) == Q_key)
					Liner_Y = -max_base_linear_speed;
				if((key_board&E_key) == E_key)
					Liner_Y = max_base_linear_speed;	
				}
			else
				Liner_Y = 0;
/***** 加速模式 *****/
			if((key_board&SHIFT_key) == SHIFT_key)
			{
				max_base_linear_speed = MAX_BASE_LINEAR_SPEED;
				max_base_rotational_speed = MAX_BASE_ROTATIONAL_SPEED;
			}
			else 
			{
				max_base_linear_speed = NORMAL_LINEAR_SPEED;
				max_base_rotational_speed = NORMAL_ROTATIONAL_SPEED;
			}

	}
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

