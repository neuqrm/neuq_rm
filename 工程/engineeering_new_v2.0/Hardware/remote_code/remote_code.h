#ifndef _REMOTE_CODE_H_
#define _REMOTE_CODE_H_
#include "sys.h"

/*
  //大疆遥控器参数范围
  ch0:364-1024-1684
	ch1:364-1024-1684
	ch2:364-1024-1684
  ch3:364-1024-1684
  s1: 1-3-2
  s2: 1-3-2
*/
	
//大疆遥控器

  #define x_CH_width            rc.ch0         //x方向通道脉宽   右摇杆左右
	#define y_CH_width            rc.ch1         //y方向通道脉宽   右摇杆上下
	#define r_CH_width            rc.ch2         //r方向通道脉宽   左摇杆左右
	#define i_CH_width            rc.ch3         //云台俯仰通道脉宽 左摇杆上下
	#define LEFT_LEVER            rc.s1          //左拨杆
	#define keyboard_mode         2   					 //键盘控制模式			s1拨到最下
	#define catch_mineral_up      3  				     //矿石抓取模式正序    s1拨到中间   1 3  2      最上云台  中间底盘  最下陀螺
	#define catch_mineral_down    1              //矿石抓取模式倒序    s1最上  1
	#define RIGHT_LEVER           rc.s2          //右拨杆
	
	/*以下定义中的数值均通过watch中观察各通道值所得*/
	
  //各方向初始值
	#define x_initial_value       1024            
	#define y_initial_value       1024
	#define r_initial_value       1024
	#define i_initial_value       1024
	//各方向最大最小值
	#define x_max_value           1684             
	#define x_min_value           364
	#define y_max_value           1684
	#define y_min_value           364
	#define r_max_value           1684
	#define r_min_value           364
	#define i_max_value           1684
	#define i_min_value           364
	//阈值上下限
	#define stop_max_value				2.5
	#define stop_min_value				1.5
	#define remote_max_value      4
	#define remote_min_value     0.5		//remote范围应包含stop
	
	#define mouse_x               RC_Ctl.mouse.x     //遥控器额外通道控制：鼠标键盘
	#define mouse_y               RC_Ctl.mouse.y
	#define mouse_z               RC_Ctl.mouse.z
  #define mouse_pre_left        RC_Ctl.mouse.press_l
	#define mouse_pre_right       RC_Ctl.mouse.press_r
	#define key_board             RC_Ctl.key.v

	#define W_key                0x000001
	#define S_key                0x000002
	#define ws_key               0x000003
	
	#define A_key                0x000004
	#define D_key                0x000008
	#define ad_key               0x00000C
	
	#define Q_key                0x000040
	#define E_key                0x000080
	#define qe_key               0x0000C0
	
	#define SHIFT_key            0x000010
	#define CTRL_key             0x000020
	
	#define F_key                0x000200
	#define Null_key             0x000000
	
	#define R_key                0x100000
  
	#define C_key                0x200000


	#define remote_ch_init() \
do{ \
		rc.ch0=1024;\
		rc.ch1=1024;\
		rc.ch2=1024;\
		rc.ch3=1024;\
}while(0)\

  #define Liner_X  Kinematics.target_velocities.linear_x
  #define Liner_Y  Kinematics.target_velocities.linear_y
  #define Angular_Z Kinematics.target_velocities.angular_z

	#define Stepper_Left_pwm     Kinematics.stepper_motor_left.pwm_pulse
	#define Stepper_Right_pwm    Kinematics.stepper_motor_right.pwm_pulse
  #define Stepper_Left_Angle   Kinematics.stepper_motor_left.pulse_num   //步进电机左侧
  #define Stepper_Right_Angle  Kinematics.stepper_motor_right.pulse_num  //步进电机右侧

	#define Handle_Left_Angel    Kinematics.handle_L.target_angle
	#define Handle_Right_Angle   Kinematics.handle_R.target_angle
	#define Handle_Left_Angular  Kinematics.handle_L.target_angular
	#define Handle_Right_Angular Kinematics.handle_R.target_angular
  #define Rescue_Claw_Pulse    Kinematics.rescue_claw_pulse              //救援爪舵机脉宽

void Remote_Control(void);
float x_max_speed_caculator(float x);
float y_max_speed_caculator(float y);
float z_max_speed_caculator(float z);

#endif
