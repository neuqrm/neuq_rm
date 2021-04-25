#include "shoot.h"
#include "motor.h"
#include "kinematic.h"
#include "fric.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "remote_code.h"

extern float trigger_angle;
extern float fric_angular;
extern float trigger_speed;


//void shoot(int bullet_num)
//{
//	fric_angular=1200;	//摩擦轮控制mhp
//	trigger_angle+=bullet_num*bullet_angle;			//发射子弹数量对应角度mhp
//	fric_shoot_assignment();	
//}

void shoot(int bullet_num)
{
	int count=0;
	fric_angular=1295.3;	//1295.3参数可以确保不会超出热量上限制，使速度在14.3m/s左右
	fric_shoot_assignment();
	
	do{
		count++;
	}while(count==10);//为使摩擦轮先开启而设置延时
	
	count=0;
	trigger_angle=bullet_num*1774;	//1774确定为子弹单发角度(最佳角度)
	fric_shoot_assignment();
	

	motor5.round_cnt=0;
	count=0;
	
	
	
}
