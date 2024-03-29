/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
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
  
#include "bsp_debug_usart.h"
#include "debug.h"


 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
  * @param  无
  * @retval 无
  */
void Debug_USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
  RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  DEBUG_USART_PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN  ;  
  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE,DEBUG_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
  
  /* 配置串DEBUG_USART 模式 */
  /* 波特率设置：DEBUG_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
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
  USART_Init(DEBUG_USART, &USART_InitStructure); 

  /* 使能串口 */
  USART_Cmd(DEBUG_USART, ENABLE);
	
	USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);//开启相关中断
	
		//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ ;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}


/***************** 数据接收 ***************************/
void DEBUG_USART_IRQHandler(void)
{
	int16_t Temp = 0;
	
	if(USART_GetITStatus(DEBUG_USART,USART_IT_RXNE)!=RESET)
	{		
		Temp = USART_ReceiveData( DEBUG_USART );;	//将收到的数据存入缓冲区中
		DEBUG_Data_solve(Temp);
	}
}

/*****************  发送一个字符 **********************/
void DEBUG_SendByte(uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(DEBUG_USART,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
}

/*****************  发送字符串 **********************/
void DEBUG_SendString(char *str)
{
	unsigned int k=0;
  do 
  {
      DEBUG_SendByte(*(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(DEBUG_USART,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void DEBUG_SendHalfWord(uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(DEBUG_USART,temp_h);	
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(DEBUG_USART,temp_l);	
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
}
/*****************  发送一个16位数组 **********************/

void DEBUG_SendHalfString(uint8_t ch,uint16_t *number)
{
	uint8_t cntstrl,i;//,cntbitl;
	cntstrl = sizeof(number) / sizeof(number[0]);
	//cntbitl = sizeof(number);
	DEBUG_SendByte(ch);
	for (i=0;i<cntstrl;i++)
	{
		DEBUG_SendHalfWord(number[i]);
	}
}

////重定向c库函数printf到串口1，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//		/* 发送一个字节数据到串口 */
//		DEBUG_SendData(DEBUG_USART, (uint8_t) ch);
//		
//		/* 等待发送完毕 */
//		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
//	
//		return (ch);
//}

////重定向c库函数scanf到串口1，重写向后可使用scanf、getchar等函数
//int fgetc(FILE *f)
//{
//		/* 等待串口输入数据 */
//		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(DEBUG_USART);
//}
/**********无线蓝牙调史发送数据************/
void Data_Send(unsigned short int *pst)
{
  unsigned char _cnt=0;	unsigned char sum = 0;
	unsigned char data_to_send[23];         //????
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //?8?
	data_to_send[_cnt++]=(unsigned char)pst[0];  //?8?
	data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[1];
	data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[2];
	data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[3];
	data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[4];
	data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[5];
	data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[6];
	data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[7];
  data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[8];

	data_to_send[3] = _cnt-4;
	
	sum = 0;
        
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
   data_to_send[_cnt++] = sum;      
   for(unsigned char i=0;i<_cnt;i++)
   DEBUG_SendByte(data_to_send[i]);
}

/*********************************************END OF FILE**********************/
