
#include "bsp_power.h"   

void POWER_GPIO_Config(void)
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*开启LED相关的GPIO外设时钟*/
		RCC_AHB1PeriphClockCmd (POWER_GPIO_CLK, ENABLE); 

		/*选择要控制的GPIO引脚*/															   
		GPIO_InitStructure.GPIO_Pin = (POWER1_PIN|POWER2_PIN|POWER3_PIN|POWER4_PIN);	

		/*设置引脚模式为输出模式*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    
    /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    
    /*设置引脚为上拉模式，默认LED亮*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

		/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
		GPIO_Init(POWER_GPIO_PORT, &GPIO_InitStructure);	
		
		/*开启电源*/
		POWER1_OFF;POWER2_OFF;POWER3_OFF;POWER4_OFF;
		
}
