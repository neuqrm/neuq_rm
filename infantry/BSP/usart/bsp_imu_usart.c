#include "bsp_imu_usart.h"
void bsp_imu_usart_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14 |GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;         
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;    
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;	 
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;		   
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; 
	USART_InitStructure.USART_Parity=USART_Parity_No;    
	USART_InitStructure.USART_StopBits=USART_StopBits_1; 
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;  
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	USART_Init(USART6,&USART_InitStructure);
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure_usart;
	NVIC_InitStructure_usart.NVIC_IRQChannel=USART6_IRQn;
	NVIC_InitStructure_usart.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure_usart.NVIC_IRQChannelPreemptionPriority=0; 
	NVIC_InitStructure_usart.NVIC_IRQChannelSubPriority=2;		
	NVIC_Init(&NVIC_InitStructure_usart);
	USART_Cmd(USART6,ENABLE);
}

