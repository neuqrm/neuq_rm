#include "shoot.h"
#include "motor.h"
#include "kinematic.h"
#include "fric.h"
#include <math.h>
#include "remote_code.h"


void shoot(int bullet_num)
{
	fric_angular=1500;	//Ħ���ֿ���mhp
	trigger_angle+=bullet_num*1667;			//�����ӵ�������Ӧ�Ƕ�mhp
	fric_shoot_assignment();		
}

