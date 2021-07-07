
#include "limit.h"

char SuperCapVoltage = 'I';


/********************************************
 *SuperCap_Calculate 超级电容功率计算函数
 *
 *A  B C  D  E  F  G  H  I  J  K  L  M  N  O  P  Q  R  S  T  U   V   W   X   Y   Z
 *0  5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100 105 110 115 120
*/

void SuperCap_Calculate(void)
{
	uint16_t ucPowerLimmit=0;
	ucPowerLimmit = Get_Chassis_Powerlimit();
	if (ucPowerLimmit == 40)
		SuperCapVoltage = 'I';
	else if (ucPowerLimmit ==45 )
		SuperCapVoltage = 'J';
	else if (ucPowerLimmit ==50 )
		SuperCapVoltage = 'K';
	else if (ucPowerLimmit ==55 )
		SuperCapVoltage = 'L';
	else if (ucPowerLimmit ==60 )
		SuperCapVoltage = 'M';
	else if (ucPowerLimmit ==65 )
		SuperCapVoltage = 'N';
	else if (ucPowerLimmit ==70 )
		SuperCapVoltage = 'O';
	else if (ucPowerLimmit ==75 )
		SuperCapVoltage = 'P';
	else if (ucPowerLimmit ==80 )
		SuperCapVoltage = 'Q';
	else if (ucPowerLimmit ==85 )
		SuperCapVoltage = 'R';
	else if (ucPowerLimmit ==90 )
		SuperCapVoltage = 'S';
	else if (ucPowerLimmit ==95 )
		SuperCapVoltage = 'T';
	else if (ucPowerLimmit ==100 )
		SuperCapVoltage = 'U';
	else if (ucPowerLimmit ==105 )
		SuperCapVoltage = 'V';
	else if (ucPowerLimmit ==110)
		SuperCapVoltage = 'W';
}

/********************************************
 *SHoot_Calculate 射速射频计算函数
 *子弹初速度 15 18 30
 *热量和冷却 限制拨弹轮转速
 *
*/
uint16_t ucFricPluse=1270;
struct ShootLimitDataSTC  ShootLimitData={1270,10};
void Shoot_Calculate(void)
{
	uint16_t ucTriggerRpm=10;
	uint16_t ShootSpeedLimit=15,ShootCoolLimit=15,ShootHeatLimit=50;
	ShootSpeedLimit = Get_Shoot_Speedlimit();
	ShootCoolLimit = Get_Shoot_Coolrate();
	ShootHeatLimit = Get_Shoot_Heatlimit();
	if (ShootSpeedLimit==15)
		ucFricPluse =FricPluse_15;
	else if (ShootSpeedLimit==18)
		ucFricPluse = FricPluse_18;
	else if (ShootSpeedLimit==30)
		ucFricPluse = FricPluse_30;
//	else
//		ucFricPluse =FricPluse_15;
	
	if (ShootCoolLimit==10)
		ucTriggerRpm = TriggerRpm_10;
	else if (ShootCoolLimit==15)
		ucTriggerRpm = TriggerRpm_15;
	else if (ShootCoolLimit==20)
		ucTriggerRpm = TriggerRpm_20;
	else if (ShootCoolLimit==25)
		ucTriggerRpm = TriggerRpm_25;
	else if (ShootCoolLimit==30)
		ucTriggerRpm = TriggerRpm_30;
	else if (ShootCoolLimit==35)
		ucTriggerRpm = TriggerRpm_35;
	else if (ShootCoolLimit==40)
		ucTriggerRpm = TriggerRpm_40;
	else if (ShootCoolLimit==60)
		ucTriggerRpm = TriggerRpm_60;
//	else
//		ucTriggerRpm = TriggerRpm_10;
	
	ShootLimitData.LimitFricPluse = ucFricPluse;
	ShootLimitData.LimitTriggerRpm = ucTriggerRpm;
}


void LimitCtrl(void)
{
	SuperCap_Calculate();
	Shoot_Calculate();
}



