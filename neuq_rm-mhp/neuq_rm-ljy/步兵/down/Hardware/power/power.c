#include "power.h"
#include "stm32f4xx_gpio.h"

void power_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	GPIO_InitStructure.GPIO_Pin=POWER_PIN;
	GPIO_InitStructure.GPIO_Mode=POWER_MODE;
	GPIO_InitStructure.GPIO_OType=POWER_OTYPE;
	GPIO_InitStructure.GPIO_PuPd=POWER_PuPd;
	GPIO_InitStructure.GPIO_Speed=POWER_SPEED;
	GPIO_Init(POWER_PORT,&GPIO_InitStructure);
	GPIO_ResetBits(POWER_PORT,POWER_PIN);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode=POWER_MODE;
	GPIO_InitStruct.GPIO_OType=POWER_OTYPE;
	GPIO_InitStruct.GPIO_PuPd=POWER_PuPd;
	GPIO_InitStruct.GPIO_Speed=POWER_SPEED;
	GPIO_Init(GPIOG,&GPIO_InitStruct);
	GPIO_SetBits(GPIOG,GPIO_Pin_13);
}



void power_open_all()
{
	GPIO_SetBits(POWER_PORT,POWER_PIN);
}



void power_close_all()
{
	GPIO_ResetBits(POWER_PORT,POWER_PIN);
}



void power_close_motor(int motor_num)
{
	switch(motor_num)
	{
		case M1:GPIO_ResetBits(POWER_PORT,M1_PIN);break;
		case M2:GPIO_ResetBits(POWER_PORT,M2_PIN);break;
		case M3:GPIO_ResetBits(POWER_PORT,M3_PIN);break;
		case M4:GPIO_ResetBits(POWER_PORT,M4_PIN);break;
		default:power_close_all();break;
	}
}
								 


void power_open_motor(int motor_num)
{
	switch(motor_num)
	{
		case M1:GPIO_SetBits(POWER_PORT,M1_PIN);break;
		case M2:GPIO_SetBits(POWER_PORT,M2_PIN);break;
		case M3:GPIO_SetBits(POWER_PORT,M3_PIN);break;
		case M4:GPIO_SetBits(POWER_PORT,M4_PIN);break;
		default:power_open_all();break;
	}
}



