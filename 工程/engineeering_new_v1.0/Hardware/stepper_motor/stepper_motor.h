#ifndef _STEPPER_MOTOR_H
#define _STEPPER_MOTOR_H

extern int left_motor_pulse;
extern int right_motor_pulse;


void TIM_Stepper_Motor_PWM_Init(void);
void Stepper_Motor_EN_GPIO_Init(void);
void Stepper_Motor_DIR_GPIO_Init(void);
void stepper_motor_control(int left_motor_angle,int right_motor_angle);


#endif
