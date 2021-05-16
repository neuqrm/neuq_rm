#ifndef __ENCODER_H
#define	__ENCODER_H
#include "stm32f4xx.h"


#define	ENCODER_PPR 256          //编码器的线数

#define L_ENCODER_PORT 		GPIOA
#define L_ENCODER_PIN 		GPIO_Pin_4 | GPIO_Pin_5

#define R_ENCODER_PORT 		GPIOA
#define R_ENCODER_PIN 		GPIO_Pin_1 | GPIO_Pin_2


void TIM5_Int_Init(u16 arr,u16 psc);
void Encoder_Init_TIM4(u16 arr,u16 psc);
int Read_Speed(int TIMx);
u32 Read_Encoder(int TIMx);

void light_encoder_Init(void);
void get_light_encoder_read_left(void);
void get_light_encoder_read_right(void);
u32 get_light_speed(int TIMx);

void ENCODER_Init(void);
u32 get_left_encoder_speed(void);
u32 get_right_encoder_speed(void);

/*#define dji_remote_assignment() \
do{ \
	Kinematics.target_velocities.linear_x=x_speed; \
    Kinematics.target_velocities.linear_y=y_speed; \
	Kinematics.target_velocities.angular_z=r_speed; \
    Kinematics.trigger.target_angular=trigger_speed; \
	Kinematics.fric1.target_angular=fric_angular;     \
	Kinematics.fric2.target_angular=fric_angular;     \
}while(0)      
*/

#define ENCODER_EN   0    //1表示使用光电门 0表示使用霍尔

#endif
