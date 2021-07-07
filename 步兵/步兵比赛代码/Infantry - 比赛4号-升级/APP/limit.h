#ifndef __LIMIT_H
#define	__LIMIT_H

#include "bsp_supercap_usart.h"
#include "referee.h"

extern char SuperCapVoltage;

extern void SuperCap_Calculate(void);		//�������ݿ��ƹ��ʼ���
extern void Shoot_Calculate(void);			//����������Ƽ���

extern void LimitCtrl(void);			//����ϵͳ������������

struct ShootLimitDataSTC 
{
	uint16_t LimitFricPluse;		//Ħ�����ٶ�
	uint16_t LimitTriggerRpm;		//�������ٶ�
};	//3508��������

extern struct ShootLimitDataSTC  ShootLimitData;	//�������
enum FricLevel
{
	FricPluse_15 = 1300,
	FricPluse_18 = 1350,
	FricPluse_30 = 1610,
};//ÿ�����ٶ�Ӧ��Ħ���ֿ�������

enum TriggerLevel
{
	TriggerRpm_10 = 10,
	TriggerRpm_15 = 15,
	TriggerRpm_20 = 20,
	TriggerRpm_25 = 25,
	TriggerRpm_30 = 30,
	TriggerRpm_35 = 35,
	TriggerRpm_40 = 40,
	TriggerRpm_60 = 60,
};//ÿ����Ƶ��Ӧ�Ĳ�����ת��

#endif /* __LIMIT_H */
