#include "bsp_imu_usart.h"

void bsp_imu_usart_init(void)  //串口8
	{
 
	//配置中断优先分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//声明一个GPIO结构体变量
	GPIO_InitTypeDef GPIO_InitStructure;
	//声明一个USART结构体变量
	USART_InitTypeDef USART_InitStructure;
	//使能USART1外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);
	//使能GPIO外设时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	//定义GPIO结构体变量，复用该IO口作为串口
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 |GPIO_Pin_1;//两个IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;         //复用模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;    //50MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;		   //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	//定义USART结构体变量
	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//硬件流控制   
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; //收发模式
	USART_InitStructure.USART_Parity=USART_Parity_No;    //无校准
	USART_InitStructure.USART_StopBits=USART_StopBits_1; //1位停止位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;  //字长为8
	//明确IO为何功能进行配置
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_UART8); //
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_UART8);
	
	//串口进行初始化
	USART_Init(UART8,&USART_InitStructure);
	//配置串口接收中断
	USART_ITConfig(UART8,USART_IT_RXNE,ENABLE);
    //串口中断优先组结构体变量声明
	NVIC_InitTypeDef NVIC_InitStructure_usart;
	//定义该串口中断优先组分组
	NVIC_InitStructure_usart.NVIC_IRQChannel=UART8_IRQn;
	NVIC_InitStructure_usart.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure_usart.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级设置
	NVIC_InitStructure_usart.NVIC_IRQChannelSubPriority=2;		//响应优先级设置
	//初始化该优先级分组
	NVIC_Init(&NVIC_InitStructure_usart);
	//使能串口1
	USART_Cmd(UART8,ENABLE);     
}
