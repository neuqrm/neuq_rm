#ifndef _FS_REMOTE_H
#define _FS_REMOTE_H
#include "sys.h"

extern u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)

//����ṹ��
typedef struct{
	
	//��ͨ��
	int CH1;
	int CH2;
	int CH3;
	int CH4;
	int CH5;
	int CH6;
	int CH7;
	int CH8;
	
}Pulse_Width_CH;

extern Pulse_Width_CH Pulse_Width;

void FS_Remote_Init(void);			//��˹ң������ʼ��



#endif

