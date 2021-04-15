#ifndef _MODE_H
#define _MODE_H
//extern int vpid_out_max;
typedef enum  chassis_mode_t
{ 
	 chassis_normal_mode=0,
	 chassis_dodge_mode=1,
   chassis_motion_mode=2,
	 chassis_follow_mode=3,
}chassis_mode_t;


typedef enum gimbal_mode_t
{ 
  gimbal_auto_mode=0,
	gimbal_load_mode=1,
	gimbal_turn_mode=2,
	gimbal_side_mode=3,
	gimbal_pwm_mode=4,
	gimbal_can_mode=5,
}gimbal_mode_t;

 typedef enum  gimbal_loop_t
{ 
	position_loop=1,
	speed_loop=0,
}gimbal_loop_t;

 typedef enum  fric_mode_t
{ 
	fric_hand_mode=0,//手动单发
	fric_Shand_mode=1, //手动三连发   
	fric_auto_mode=2,
}fric_mode_t;

 typedef enum  control_mode_t
{ 
	auto_control=0,
	DJi_Remote_Control=1,
}control_mode_t;


extern enum  chassis_mode_t  chassis_modes;
extern enum  gimbal_mode_t   gimbal_modes;
extern enum  fric_mode_t     fric_modes;
extern enum  control_mode_t  control_mode;
extern enum  gimbal_loop_t   gimbal_loop;
void  chassis_behavior(void);
void  gimbal_behavior(void);
void  fric_behavior(void);
void  all_behavior(void);
void  mode_init(void);

#endif
