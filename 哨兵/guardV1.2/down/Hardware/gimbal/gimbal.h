#ifndef __PWM_H
#define __PWM_H

//pwmģʽ�µĽǶȻ���
#define BASIC_YAW_ANGLE_PWM       1290       //yaw��Ļ�׼λ�õ�����
#define BASIC_PITCH_ANGLE_PWM     1500       //pitch���׼λ�õ�����
#define BASIC_YAW_ANGLE_CAN       2700       //yaw��Ļ�׼λ�õı������Ƕ�ֵ
#define BASIC_PITCH_ANGLE_CAN     3400       //pitch���׼λ�õı������Ƕ�ֵ
#define GM6020_ENCODER_ANGLE      8191       //��̨�������������ֵ
void TIM1_GIMBAL_Init(void);
extern float pwm_pulse_p;
extern float pwm_pulse_y;
#endif
