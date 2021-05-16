
#include "bsp_supercap_usart.h"
#include "referee.h"

 /**
  * @brief  CAP_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
  * @param  无
  * @retval 无
  */
void CAP_USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
  RCC_AHB1PeriphClockCmd(CAP_USART_RX_GPIO_CLK|CAP_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  CAP_USART_PeriphClockCmd(CAP_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CAP_USART_TX_PIN  ;  
  GPIO_Init(CAP_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CAP_USART_RX_PIN;
  GPIO_Init(CAP_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(CAP_USART_RX_GPIO_PORT,CAP_USART_RX_SOURCE,CAP_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(CAP_USART_TX_GPIO_PORT,CAP_USART_TX_SOURCE,CAP_USART_TX_AF);
  
  /* 配置串CAP_USART 模式 */
  /* 波特率设置：CAP_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = CAP_USART_BAUDRATE;
  /* 字长(数据位+校验位)：8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* 停止位：1个停止位 */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* 校验位选择：不使用校验 */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* 硬件流控制：不使用硬件流 */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USART模式控制：同时使能接收和发送 */
  USART_InitStructure.USART_Mode = USART_Mode_Tx ;//| USART_Mode_Tx;
  /* 完成USART初始化配置 */
  USART_Init(CAP_USART, &USART_InitStructure); 

  /* 使能串口 */
  USART_Cmd(CAP_USART, ENABLE);
	
	USART_ITConfig(CAP_USART, USART_IT_RXNE, ENABLE);//开启相关中断
	
		//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = CAP_USART_IRQ ;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}




/*****************  发送一个字符 **********************/
void CAP_SendByte(uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(CAP_USART,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(CAP_USART, USART_FLAG_TXE) == RESET);
}
  //超级电容等级设置  初始为5
char cap_level='5';

void supercap_Init(void)
{
	CAP_USART_Config();                //初始化超级电容的串口
	CAP_SendByte(cap_level);             //初始化超级电容的等级
}

/********         使用超级电容          *********/



void use_supercap(void)
{
	uint8_t robot_level;	
	
	robot_level = Get_Robot_Stats();
	   
	if(robot_level== 1) 
	{
		cap_level='5';
		CAP_SendByte(cap_level);
	}
	else if(robot_level==2)
	{
		cap_level='6';
		CAP_SendByte(cap_level);
	}
	else if(robot_level==3)
	{
		cap_level='6';
		CAP_SendByte(cap_level);
	}			
}