#ifndef __PWM_H
#define __PWM_H

//pwmģʽ�µĽǶȻ���
#define BASIC_YAW_ANGLE_PWM       1290       //yaw��Ļ�׼λ�õ�����
#define BASIC_PITCH_ANGLE_PWM     1500       //pitch���׼λ�õ�����
#define BASIC_YAW_ANGLE_CAN       1140//1553       //yaw��Ļ�׼λ�õı������Ƕ�ֵ
#define BASIC_PITCH_ANGLE_CAN     5410//6000       //pitch���׼λ�õı������Ƕ�ֵ

void TIM1_GIMBAL_Init(void);
extern float pwm_pulse_p;
extern float pwm_pulse_y;
#endif
