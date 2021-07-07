#ifndef __JSON_H
#define	__JSON_H

#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include <jansson.h>
#include "basicdata.h"
#include "motion.h"


#define MAX_LENGTH 150
#define JASON_START	0x0F
#define JASON_STOP	0xF0
#define JASON_GIMBALL_ANGLE	0x01
#define JASON_GIMBALL_ACTION	0x02
#define JASON_CHASSIC_SPEED		0x04
#define JASON_GIMBALL_SHOOT		0x80

extern Chassis_Rpm_stc  	Json_Chassis_Rpm;
extern Gimbal_Yaw_stc  		Json_Gimbal_Yaw;
extern Gimbal_Pitch_stc 	Json_Gimbal_Pitch;
extern char Json_Buffer[MAX_LENGTH]; 
extern void JASON_Data_Solve(uint8_t uctemp);

#endif /* __JSON_H */
