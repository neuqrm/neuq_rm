#include "shoot.h"
#include "motor.h"
#include "kinematic.h"
#include "fric.h"
#include <math.h>
#include "remote_code.h"


void shoot(int bullet_num)
{
	fric_angular=1500;	//摩擦轮控制mhp
	trigger_angle+=bullet_num*1667;			//发射子弹数量对应角度mhp
	fric_shoot_assignment();		
}

