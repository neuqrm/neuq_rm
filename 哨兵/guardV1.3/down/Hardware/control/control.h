#ifndef __CONTROL_H
#define	__CONTROL_H

#include "mode.h"    //改变角度环和速度环的参数
#include "kinematic.h"	//调用接口赋值角速度，调用实际速度接口

//宏定义，参数设置接口
#define	cruise	1    //巡航
#define	cruise_left_speed	20   //巡航左端速度，顺时针速度为正
#define	cruise_right_speed	-20   //巡航右端速度，逆时针速度为负
#define cruise_left_speed_1 -15  //pitch轴上端速度 , 向上为正
#define cruise_right_speed_1 15  //pitch轴下端速度 ， 向下为负
#define left_yaw	360   //左边界   逆时针由右向左转
#define right_yaw 240	//右边界

extern int control_flag;


void cruise_mode(void);

#endif
