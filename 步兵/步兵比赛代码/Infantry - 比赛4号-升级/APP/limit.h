#ifndef __LIMIT_H
#define	__LIMIT_H

#include "bsp_supercap_usart.h"
#include "referee.h"

extern char SuperCapVoltage;

extern void SuperCap_Calculate(void);		//超级电容控制功率计算
extern void Shoot_Calculate(void);			//射击参数控制计算

extern void LimitCtrl(void);			//裁判系统数据限制升级

struct ShootLimitDataSTC 
{
	uint16_t LimitFricPluse;		//摩擦轮速度
	uint16_t LimitTriggerRpm;		//播弹轮速度
};	//3508反馈报文

extern struct ShootLimitDataSTC  ShootLimitData;	//射击限制
enum FricLevel
{
	FricPluse_15 = 1300,
	FricPluse_18 = 1350,
	FricPluse_30 = 1610,
};//每个射速对应的摩擦轮控制脉冲

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
};//每个射频对应的拨弹轮转速

#endif /* __LIMIT_H */
