#ifndef _MY_INIT_H
#define _MY_INIT_H
#include "delay.h"
#include "mode.h"
#include "Tim3_Events.h"
#include "led.h"
#include "bsp_debug_usart.h"
#include "remote_code.h"
#include "motor.h"
#include "kinematic.h"
#include "fric.h"
#include "json.h"
#include <jansson.h>
#include <string.h>
#include "gimbal.h"
#include "imuReader.h"
#include "stm32f4xx_conf.h"



/************ͨ����flag****************/
extern uint8_t flag_command_recieved;
extern uint8_t flag_command_recieved1;
extern uint8_t flag_command_recieved2;
extern uint8_t flag_command_recieved3;
extern uint8_t flag_command_recieved4;
extern uint8_t flag_command_recieved5;
/**************************************/

/************����ʹ�ܱ�ʶ��*************/
#define MSG_SEND_EN 0      // 1Ϊʹ�ܣ�0Ϊʧ��
/**************************************/

void All_Init(void);			//���������в�����ʼ������



#endif
