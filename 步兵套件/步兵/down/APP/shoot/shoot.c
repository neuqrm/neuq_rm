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
//	fric_angular=1200;	//Ħ���ֿ���mhp
//	trigger_angle+=bullet_num*bullet_angle;			//�����ӵ�������Ӧ�Ƕ�mhp
//	fric_shoot_assignment();	
//}

void shoot(int bullet_num)
{
	int count=0;
	fric_angular=1295.3;	//1295.3��������ȷ�����ᳬ�����������ƣ�ʹ�ٶ���14.3m/s����
	fric_shoot_assignment();
	
	do{
		count++;
	}while(count==10);//ΪʹĦ�����ȿ�����������ʱ
	
	count=0;
	trigger_angle=bullet_num*1774;	//1774ȷ��Ϊ�ӵ������Ƕ�(��ѽǶ�)
	fric_shoot_assignment();
	

	motor5.round_cnt=0;
	count=0;
	
	
	
}
