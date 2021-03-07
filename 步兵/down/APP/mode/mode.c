/**
  ******************************************************************************
  * @file    Project/APP/mode.c 
  * @author  Junyu Luo
  * @version V1.0.0
  * @date    3.2021
  * @brief   模式控制
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/
#include "json.h" 
#include "mode.h"
#include "motor.h"

enum chassis_mode_t chassis_modes;
enum gimbal_mode_t gimbal_modes;
enum fric_mode_t fric_modes;
enum control_mode_t  control_mode;

void chassis_behavior(void)
{

	switch(chassis_modes)
	{
		case chassis_normal_mode:
					
		break;
		
		case 	chassis_dodge_mode:
			
		break;
		
		case chassis_motion_mode:
					
		break;
		
		case  chassis_follow_mode:
			
		break;
		
		default:
		break;
	}
}

void  gimbal_behavior(void)
{
switch(gimbal_modes)
	{
	case gimbal_auto_mode:
		
	break;
	
	case gimbal_load_mode:
		
	break;
	
	case gimbal_turn_mode:
		
	break;
	
	case gimbal_side_mode:
		
	break;
	
	default:
	break;
	
   }

}

void  fric_behavior(void)
{
 switch(fric_modes)
 {
	 case fric_hand_mode:
		 
	 break;
	 
	 case fric_Shand_mode:
		 
	 break;
	 
	 case  fric_auto_mode:
		 
	 break;
	 
	 default:
	 break;
 
 }
}

void  all_behavior()
{
   chassis_behavior();
	 gimbal_behavior();
	 fric_behavior();
}

void mode_init()
{
	//通信模式选择
	control_mode = DJi_Remote_Control;
	//云台模式选择
  gimbal_modes = gimbal_pwm_mode;
	
	//底盘模式选择
	
	//摩擦轮模式选择
	
}
