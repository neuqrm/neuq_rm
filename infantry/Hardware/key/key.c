#include "key.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void key_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;		//输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;		//下拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

u8 key_press()
{
	static u8 key_state = 0,key_last_state = 0;
	u8 data_back = 0;
	
	key_state = PBin(2);		
	if(key_state==1)
	{
		if(key_last_state == key_state)		
		{
			key_last_state = key_state;
			data_back = 1;
		}
		else
		{
			key_last_state = key_state;
			data_back = 0;
		}
	}
	else
		data_back = 0;
	
	return data_back;	
}



