#ifndef _REMOTE_CODE_H_
#define _REMOTE_CODE_H_
#include "sys.h"

//Control_Mode第1位   FS为0   DJi为1
#define FS_Remote_Control			0x0
#define DJi_Remote_Control		0x1

//Control_Mode第2位  auto为1   remote为0
#define auto_control					0x2
#define remote_control        0x1
#define ROS_control						0x4

//Control_Mode第1位遥控器选择，第二位模式选择
#define control_mode     (auto_control | DJi_Remote_Control)			//自动+大疆遥控器

extern u8 Control_Mode;

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
	#define Remote_control_mode   rc.s1
	#define dance_CH_width        2   					 //跳舞脉宽				s1拨到最下
	#define chassis_CH_width      3  				     //遥控底盘    s1拨到中间   1 3  2      最上云台  中间底盘  最下陀螺
	#define gimbal_CH_width       1              //遥控云台    s1最上  1
	#define trigger_CH_width      rc.s2          //拨弹轮控制
	
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


void Remote_Control(void);
float x_max_speed_caculator(float x);
float y_max_speed_caculator(float y);
float z_max_speed_caculator(float z);


#endif
