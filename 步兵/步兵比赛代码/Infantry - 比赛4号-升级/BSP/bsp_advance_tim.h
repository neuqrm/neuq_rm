#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "stm32f4xx.h"

/* 定时器 */
#define TIM1_PWM12_GPIO_CLOCK          	RCC_AHB1Periph_GPIOA
#define TIM1_PWM34_GPIO_CLOCK          	RCC_AHB1Periph_GPIOE
#define TIM8_PWM_GPIO_CLOCK          		RCC_AHB1Periph_GPIOI



/* TIM1通道1\2\3\4输出引脚 */
#define TIM1_PWM12_GPIO_PORT        GPIOA 
#define TIM1_PWM1_PIN             	GPIO_Pin_8          
#define TIM1_PWM2_PIN             	GPIO_Pin_9   
#define TIM1_PWM34_GPIO_PORT        GPIOE 
#define TIM1_PWM3_PIN             	GPIO_Pin_13   
#define TIM1_PWM4_PIN             	GPIO_Pin_14 

/* TIM8通道1\2\3\4输出引脚 */
#define TIM8_PWM1_PIN             	GPIO_Pin_5                      
#define TIM8_PWM2_PIN								GPIO_Pin_6
#define TIM8_PWM3_PIN								GPIO_Pin_7
#define TIM8_PWM4_PIN								GPIO_Pin_2
#define TIM8_PWM_GPIO_PORT        	GPIOI 

/* TIM1通道1\2\3\4复用引脚 */
#define	TIM1_PWM1_SOURCE						GPIO_PinSource8
#define	TIM1_PWM2_SOURCE						GPIO_PinSource9
#define	TIM1_PWM3_SOURCE						GPIO_PinSource13
#define	TIM1_PWM4_SOURCE						GPIO_PinSource14

/* TIM8通道1\2\3\4复用引脚 */
#define	TIM8_PWM1_SOURCE						GPIO_PinSource5
#define	TIM8_PWM2_SOURCE						GPIO_PinSource6
#define	TIM8_PWM3_SOURCE						GPIO_PinSource7
#define	TIM8_PWM4_SOURCE						GPIO_PinSource2

extern uint16_t Helm_pluse;// = 1000;
void TIM8_Configuration(void);
void TIM1_Configuration(void);
#endif /* __ADVANCE_TIM_H */

