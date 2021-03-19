#include "shoot.h"
#include "motor.h"
#include "kinematic.h"
#include "fric.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "remote_code.h"

float trigger_angle=0;
extern float fric_angular;
extern float trigger_speed;
int wave_value=1;
int count=0;

void shoot(int bullet_num)
{
	fric_angular=1200;	//摩擦轮控制mhp
	fric_shoot_assignment();
	trigger_angle=bullet_num*360/7;											//发射子弹数量对应角度mhp
	trigger_angle_control(trigger_angle);
	
	while(motor5.actual_angle==motor5.last_bullet_angle)    				//堵转后抖动处理mhp
	{ 
		trigger_speed =pow(-1,wave_value)*50;
		trigger_control(trigger_speed);
		if(motor5.actual_angle==motor5.last_bullet_angle)
			break;
	}
	
	motor5.last_bullet_angle=motor5.actual_angle;
	
	do{
		count++;
	}while(count==300);														//摩擦轮延时
	
}

