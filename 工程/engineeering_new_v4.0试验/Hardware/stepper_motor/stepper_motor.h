#ifndef _STEPPER_MOTOR_H
#define _STEPPER_MOTOR_H

#define k 100


extern int distance;

extern int left_motor_pulse;
extern int right_motor_pulse;

void TIM4_NVIC_Init(void);
void TIM_Stepper_Motor_PWM_Init(void);
void Stepper_Motor_EN_GPIO_Init(void);
void Stepper_Motor_DIR_GPIO_Init(void);
//void stepper_motor_control(int left_motor_angle,int right_motor_angle);

void stepper_motor_up(void);
void stepper_motor_down(void);
void stepper_motor_off(void);
void Stepper_Motor_VCC_GPIO_Init(void);

void stepper_motor_config(int left_motor_round,int right_motor_round,int subdivision_value);
void stepper_motor_control(int distance);


#endif
