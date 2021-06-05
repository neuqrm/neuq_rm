#ifndef _MY_INIT_H
#define _MY_INIT_H
#include "delay.h"
#include "Tim3_Events.h"
#include "led.h"
#include "bsp_debug_usart.h"
#include "remote_code.h"
#include "motor.h"
#include "kinematic.h"
#include <string.h>
#include "stm32f4xx_conf.h"
#include "power.h"

void software_init(void);  
void hardware_init(void);  
void All_Init(void);
#endif
