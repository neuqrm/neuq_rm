#ifndef __CONTROL_H
#define	__CONTROL_H

#include "mode.h"    //�ı�ǶȻ����ٶȻ��Ĳ���
#include "kinematic.h"	//���ýӿڸ�ֵ���ٶȣ�����ʵ���ٶȽӿ�

//�궨�壬�������ýӿ�
#define	cruise	1    //Ѳ��
#define	cruise_left_speed	20   //Ѳ������ٶȣ�˳ʱ���ٶ�Ϊ��
#define	cruise_right_speed	-20   //Ѳ���Ҷ��ٶȣ���ʱ���ٶ�Ϊ��
#define cruise_left_speed_1 -15  //pitch���϶��ٶ� , ����Ϊ��
#define cruise_right_speed_1 15  //pitch���¶��ٶ� �� ����Ϊ��
#define left_yaw	360   //��߽�   ��ʱ����������ת
#define right_yaw 240	//�ұ߽�

extern int control_flag;


void cruise_mode(void);

#endif
