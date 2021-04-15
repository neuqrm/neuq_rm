#ifndef _STEERING_ENGINE_H
#define _STEERING_ENGINE_H

extern float pwm_steering_engine;
extern int steering_engine;

void TIM_Steering_Engine_PWM_Init(void);
void steering_engine_on(void);
void steering_engine_off(void);

#endif
