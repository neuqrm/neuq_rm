#ifndef __PWM_H
#define __PWM_H

//pwm模式下的角度换算
#define BASIC_YAW_ANGLE_PWM       1290       //yaw轴的基准位置的脉宽
#define BASIC_PITCH_ANGLE_PWM     1500       //pitch轴基准位置的脉宽
#define BASIC_YAW_ANGLE_CAN       2047       //yaw轴的基准位置的编码器角度值
#define BASIC_PITCH_ANGLE_CAN     4095       //pitch轴基准位置的编码器角度值

void TIM1_GIMBAL_Init(void);
void TIM2_Init(void);
extern float pwm_pulse_p;
extern float pwm_pulse_y;
extern float pwm_pulse_1;
extern float pwm_pulse_2;
#endif
