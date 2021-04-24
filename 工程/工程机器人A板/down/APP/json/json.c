/**
  ******************************************************************************
  * @file    Project/APP/json.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   json文件
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include <string.h>
#include <stdio.h>
#include <jansson.h>
#include "json.h" 
#include "motor.h"
#include "kinematic.h"
#include "mode.h"
#include "bsp_debug_usart.h"
#include "remote_code.h"
#include "gimbal.h"
#include "delay.h"
#include "stm32f4xx_it.h"

/********信息发送部分********/
/**
  *@brief 步兵信息发送
  */
void send_infantry_info_by_json(void)
{  	
	json_t *root;
	char *out;           
	root = json_pack("[{sfsfsfsfsfsfsf}[fffffff]]",\
						"wheel1.rpm", (Kinematics.wheel1.actual_speed.rpm),\
						"wheel2.rpm", (Kinematics.wheel2.actual_speed.rpm ),\
					  "wheel3.rpm", (Kinematics.wheel3.actual_speed.rpm),\
	          "wheel4.rpm", (Kinematics.wheel4.actual_speed.rpm),\
						"linear_x", (Kinematics.actual_velocities.linear_x),\
	          "linear_y", (Kinematics.actual_velocities.linear_y),\
	          "angular_z", (Kinematics.actual_velocities.angular_z),\
	          (Kinematics.wheel1.actual_speed.rpm),\
						(Kinematics.wheel2.actual_speed.rpm),\
	          (Kinematics.wheel3.actual_speed.rpm),\
						(Kinematics.wheel4.actual_speed.rpm),\
            (Kinematics.actual_velocities.linear_x),\
	          (Kinematics.actual_velocities.linear_y),\
          	(Kinematics.actual_velocities.angular_z));
					
					
 	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);
}

/**
  *@brief 底盘信息发送
  */
void send_chassis_info_by_json(void)
{
	json_t *root;
	char *out;           //
	root = json_pack("[{sfsfsfsfsf}[fffff]]",\
						"linear_x", (Kinematics.actual_velocities.linear_x),\
						"linear_y", (Kinematics.actual_velocities.linear_y),\
						"angular_z", (Kinematics.actual_velocities.angular_z),\
	          "pitch_angle", (motor6.actual_angle),\
	          "yaw_angle", (motor5.actual_angle),\
						(Kinematics.actual_velocities.linear_x),\
						(Kinematics.actual_velocities.linear_y),\
						(Kinematics.actual_velocities.angular_z),\
	          (motor6.actual_angle),\
          	(motor5.actual_angle));
	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);
}

/**
  *@brief 云台信息发送
  */
void send_gimbal_info_by_json(void)   
{
  json_t *root;
	char *out;           
	root = json_pack("[{sfsf}[ff]]",\
					  "yaw_angular", (Kinematics.yaw.actual_angular),\
						"pitch_angular", (Kinematics.pitch.actual_angular),\
						(Kinematics.yaw.actual_angular),\
						(Kinematics.pitch.actual_angular));   //(int),
	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);

}

/**
  *@brief 摩擦轮信息发送 
  */
void send_fric_info_by_json()   
{
//����   
Kinematics.fric.actual_angular=7;
json_t *root;
	char *out;           //
	root = json_pack("[{sf}[f]]",\
						
					  "fric_angular", (Kinematics.fric.actual_angular),\
						
						(Kinematics.fric.actual_angular));   //(int),
						
	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);
}

/**********信息解析部分*************/
	float tmp_getx;
	float tmp_gety;

  float pitch_angle;
	float yaw_angle;


/**
  *@brief 接受底盘数据
  */
void resolve_json_chassis_command(void)
{
	json_t *root;
	json_t *chassis_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error); //
	chassis_obj = json_object_get( root, "chassis" );  //Get a value corresponding to key from object
	item_obj = json_array_get( chassis_obj, 0 );//Returns the element in array at position index
	Kinematics.target_velocities.linear_x =5.0f*json_integer_value(item_obj);	//real
	item_obj = json_array_get( chassis_obj, 1 );
	Kinematics.target_velocities.linear_y = 5.0f*json_integer_value(item_obj);
	item_obj = json_array_get( chassis_obj, 2 );
	Kinematics.target_velocities.angular_z = 0.8f*json_integer_value(item_obj);
	json_decref(item_obj); //Decrement the reference count of json. As soon as a call to json_decref() drops the reference count to zero, the value is destroyed and it can no longer be used.
	json_decref(chassis_obj);
	json_decref(root);
}

/**
  *@brief 接受云台数据
  */
void resolve_json_gimbal_speed_command()
{ 
	json_t *root;
	json_t *gimbal_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	gimbal_obj = json_object_get( root, "gimbal" );
	item_obj = json_array_get( gimbal_obj, 0 );
  Kinematics.yaw.target_angular=1.0f*json_integer_value(item_obj); //10000;
	//item_obj = json_array_get( gimbal_obj, 1 );
	//Kinematics.pitch.target_angular=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(gimbal_obj);
	json_decref(root);
}

/**
  *@brief 接受上位机调节pid参数
  */
/*void resolve_json_pidparam_command(void)   
{ 
	json_t *root;
	json_t *para_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	para_obj = json_object_get( root, "Pid_param" );
	item_obj = json_array_get( para_obj, 0 );
	Chassis.kp=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( para_obj, 1 );
	Chassis.ki=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( para_obj, 2 );
	Gimbal.kp=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( para_obj, 3 );
	Gimbal.ki=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( para_obj, 4);
	Trigger.kp=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( para_obj, 5 );
	Trigger.ki=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(para_obj);
	json_decref(root);
}
*/

void resolve_json_gimbal_angle_command(void)
{ 
	json_t *root;
	json_t *gimbal_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	gimbal_obj = json_object_get( root, "gimbal_angle" );
	item_obj = json_array_get( gimbal_obj, 0 );
	Kinematics.yaw.target_angle=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( gimbal_obj, 1 );
	Kinematics.pitch.target_angle=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(gimbal_obj);
	json_decref(root);
}

/**
  *@brief 接受摩擦轮数据
  */
int fric_1;
void resolve_json_fric_command()
	{ 
	json_t *root;
	json_t *fric_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	fric_obj = json_object_get( root, "fric_angular" );
	item_obj = json_array_get( fric_obj, 0 );
	Kinematics.fric.target_angular=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(fric_obj);
	json_decref(root);

}
	
/**
  *@brief 接受拨弹轮数据
  */
void resolve_json_trigger_command()
	{ 
	json_t *root;
	json_t *trigger_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	trigger_obj = json_object_get( root, "trigger_angular" );
	item_obj = json_array_get( trigger_obj, 0 );
	Kinematics.trigger.target_angular=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(trigger_obj);
	json_decref(root);

}
	

void resolve_json_mode_command()
{
  resolve_chassis_mode_command();
	resolve_gimbal_mode_command();
	resolve_fric_mode_command();

}
void resolve_chassis_mode_command()
{ 
  json_t *root;
	json_t *chassis_mode_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	chassis_mode_obj = json_object_get( root, "translation" );
	item_obj = json_array_get( chassis_mode_obj, 0 );
	chassis_modes = (chassis_mode_t)(json_integer_value(item_obj));
	json_decref(item_obj);
	json_decref(chassis_mode_obj);
	json_decref(root);

}
void resolve_gimbal_mode_command()
{
  json_t *root;
	json_t *gimbal_mode_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	gimbal_mode_obj = json_object_get( root, "gimbal_mode" );
	item_obj = json_array_get( gimbal_mode_obj, 0 );
	gimbal_modes = (gimbal_mode_t)(json_integer_value(item_obj));
	json_decref(item_obj);
	json_decref(gimbal_mode_obj);
	json_decref(root);

}
void resolve_fric_mode_command()
{
  json_t *root;
	json_t *fric_mode_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	fric_mode_obj = json_object_get( root, "fric_mode" );
	item_obj = json_array_get( fric_mode_obj, 0 );
	fric_modes = (fric_mode_t)(json_integer_value(item_obj));
	json_decref(item_obj);
	json_decref(fric_mode_obj);
	json_decref(root);
}




//对上位机传来的信息进行二次加工
/**
  *@brief pwm控制云台模式下的角度和脉宽换算
  */
void caclulate_pwm_pulse()
{
	float unit_pwm_pulse= (840.0f/360.0f);
	
	if(Kinematics.pitch.target_angle< (-90) && Kinematics.pitch.target_angle > 90)
	   pwm_pulse_p = (BASIC_PITCH_ANGLE_PWM +unit_pwm_pulse * Kinematics.pitch.target_angle)*1.0f;
	if(Kinematics.yaw.target_angle < (-90) && (Kinematics.yaw.target_angle > 90))
	   pwm_pulse_y = (BASIC_YAW_ANGLE_PWM +unit_pwm_pulse * Kinematics.yaw.target_angle)*1.0f;
	
}

void caclulate_handpwm_pulse()
{
  static double  yaw_pwm_pulse=1500;
  static double  pitch_pwm_pulse=1500;

	if(Kinematics.yaw.target_angular==1 && pwm_pulse_y>=1395)
	{
    yaw_pwm_pulse=yaw_pwm_pulse-1;
		pwm_pulse_y=yaw_pwm_pulse;
		delay_ms(2);
	}
	
	if(Kinematics.yaw.target_angular==-1 && pwm_pulse_y<=1605)
	{
		yaw_pwm_pulse++;
		pwm_pulse_y=yaw_pwm_pulse;
		delay_ms(2);
		
	}
	if(Kinematics.yaw.target_angular==0)
	{
		if(yaw_pwm_pulse>1500)
			 yaw_pwm_pulse=yaw_pwm_pulse-1;
		   pwm_pulse_y=yaw_pwm_pulse;
		   delay_ms(2);
		if(yaw_pwm_pulse<1500)
		  yaw_pwm_pulse++;
		  pwm_pulse_y=yaw_pwm_pulse;
		  delay_ms(2);
	}
	
	
	if(Kinematics.pitch.target_angular==1 && pwm_pulse_p<=1605)
	{
		pitch_pwm_pulse++;
		pwm_pulse_p=pitch_pwm_pulse;
		delay_ms(2);
		
	}
	if(Kinematics.pitch.target_angular==-1 && pwm_pulse_p>=1395)
	{
		pitch_pwm_pulse=pitch_pwm_pulse-1;
		pwm_pulse_p=pitch_pwm_pulse;
		delay_ms(2);
	}
	
		if(Kinematics.pitch.target_angular==0)
	{
			if(pitch_pwm_pulse>1500)
			 pitch_pwm_pulse=pitch_pwm_pulse-1;
		   pwm_pulse_p=pitch_pwm_pulse;
		   delay_ms(2);
		  if(pitch_pwm_pulse<1500)
		  pitch_pwm_pulse++;
		  pwm_pulse_p=pitch_pwm_pulse;
		  delay_ms(2);
	}

}
