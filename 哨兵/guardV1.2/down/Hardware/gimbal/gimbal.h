#ifndef __PWM_H
#define __PWM_H

//pwm模式下的角度换算
#define BASIC_YAW_ANGLE_PWM       1290       //yaw轴的基准位置的脉宽
#define BASIC_PITCH_ANGLE_PWM     1500       //pitch轴基准位置的脉宽
#define BASIC_YAW_ANGLE_CAN       2700       //yaw轴的基准位置的编码器角度值
#define BASIC_PITCH_ANGLE_CAN     3400       //pitch轴基准位置的编码器角度值
#define GM6020_ENCODER_ANGLE      8191       //云台编码器的最大数值
void TIM1_GIMBAL_Init(void);
extern float pwm_pulse_p;
extern float pwm_pulse_y;
#endif
