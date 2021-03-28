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


void shoot(int bullet_num)
{
	int count=0;
	 fric_angular=1200;	//Ħ���ֿ���mhp
	fric_shoot_assignment();
	trigger_angle+=bullet_num*1667;			//�����ӵ�������Ӧ�Ƕ�mhp
	fric_shoot_assignment();
	
	while(motor5.actual_angle==motor5.last_bullet_angle)    				//��ת�󶶶�����mhp
	{ 
		trigger_speed =pow(-1,wave_value)*50;
		trigger_control(trigger_speed);
		if(motor5.actual_angle==motor5.last_bullet_angle)
			break;
	}
	
	motor5.last_bullet_angle=motor5.actual_angle;
	
	do{
		count++;
	}while(count==300);														//Ħ������ʱ�����޸ġ�
	
}

