/**
  ******************************************************************************
  * @file    bsp_IMU_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_imu_usart.h"
#include "imu.h"


 /**
  * @brief  IMU_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
  * @param  无
  * @retval 无
  */
void IMU_USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
  RCC_AHB1PeriphClockCmd(IMU_USART_RX_GPIO_CLK|IMU_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  IMU_USART_PeriphClockCmd(IMU_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = IMU_USART_TX_PIN  ;  
  GPIO_Init(IMU_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = IMU_USART_RX_PIN;
  GPIO_Init(IMU_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(IMU_USART_RX_GPIO_PORT,IMU_USART_RX_SOURCE,IMU_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(IMU_USART_TX_GPIO_PORT,IMU_USART_TX_SOURCE,IMU_USART_TX_AF);
  
  /* 配置串IMU_USART 模式 */
  /* 波特率设置：IMU_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = IMU_USART_BAUDRATE;
  /* 字长(数据位+校验位)：8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* 停止位：1个停止位 */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* 校验位选择：不使用校验 */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* 硬件流控制：不使用硬件流 */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USART模式控制：同时使能接收和发送 */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* 完成USART初始化配置 */
  USART_Init(IMU_USART, &USART_InitStructure); 

  /* 使能串口 */
  USART_Cmd(IMU_USART, ENABLE);
	
	USART_ITConfig(IMU_USART, USART_IT_RXNE, ENABLE);//开启相关中断
	
		//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = IMU_USART_IRQ ;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}




/*****************  发送一个字符 **********************/
void IMU_SendByte(uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(IMU_USART,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(IMU_USART, USART_FLAG_TXE) == RESET);
}

/*****************  发送字符串 **********************/
void IMU_SendString(char *str)
{
	unsigned int k=0;
	uint8_t cntstrl = 0;
	cntstrl = sizeof(str) / sizeof(str[0]);
  do 
  {
      IMU_SendByte(*(str + k) );
      k++;
  } while(k < cntstrl);
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(IMU_USART,USART_FLAG_TC)==RESET)
  {}
}

void IMU_USART_IRQHandler(void)
{
	uint8_t Temp = 0;
	if(USART_GetITStatus(IMU_USART,USART_IT_RXNE)!=RESET)
	{		
		Temp = USART_ReceiveData( IMU_USART );;	//将收到的数据存入缓冲区中
		IMU_Data_Solve(Temp);
	}
	
}
/*********************************************END OF FILE**********************/
