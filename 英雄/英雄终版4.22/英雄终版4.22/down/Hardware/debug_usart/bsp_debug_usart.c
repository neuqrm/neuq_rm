/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �ض���c��printf������usart�˿ڣ��жϽ���ģʽ
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 F429 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_debug_usart.h"



 /**
  * @brief  DEBUG_USART GPIO ����,����ģʽ���á�115200 8-N-1 ���жϽ���ģʽ
  * @param  ��
  * @retval ��
  */
//void Debug_USART_Config(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//		
//  RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);

//  /* ʹ�� USART ʱ�� */
//  DEBUG_USART_PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
//  
//  /* GPIO��ʼ�� */
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  
//  /* ����Tx����Ϊ���ù���  */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN  ;  
//  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//  /* ����Rx����Ϊ���ù��� */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
//  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
//  
// /* ���� PXx �� USARTx_Tx*/
//  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE,DEBUG_USART_RX_AF);

//  /*  ���� PXx �� USARTx__Rx*/
//  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
//  
//  /* ���ô�DEBUG_USART ģʽ */
//  /* ���������ã�DEBUG_USART_BAUDRATE */
//  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
//  /* �ֳ�(����λ+У��λ)��8 */
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  /* ֹͣλ��1��ֹͣλ */
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  /* У��λѡ�񣺲�ʹ��У�� */
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  /* Ӳ�������ƣ���ʹ��Ӳ���� */
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  /* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  /* ���USART��ʼ������ */
//  USART_Init(DEBUG_USART, &USART_InitStructure); 

//  /* ʹ�ܴ��� */
//  USART_Cmd(DEBUG_USART, ENABLE);
//	
//	USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);//��������ж�
//	
//		//Usart2 NVIC ����
//	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ ;//����1�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//}




/*****************  ����һ���ַ� **********************/
void DEBUG_SendByte(uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(DEBUG_USART,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
}

/*****************  �����ַ��� **********************/
void DEBUG_SendString(char *str)
{
	unsigned int k=0;
  do 
  {
      DEBUG_SendByte(*(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(DEBUG_USART,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void DEBUG_SendHalfWord(uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(DEBUG_USART,temp_h);	
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(DEBUG_USART,temp_l);	
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
}
/*****************  ����һ��16λ���� **********************/

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
void DEBUG_USART_IRQHandler(void)
{
  uint8_t ucTemp;
	if(USART_GetITStatus(DEBUG_USART,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData( DEBUG_USART );
    USART_SendData(DEBUG_USART,ucTemp);    
	}	 
}	
////�ض���c�⺯��printf������1���ض�����ʹ��printf����
//int fputc(int ch, FILE *f)
//{
//		/* ����һ���ֽ����ݵ����� */
//		DEBUG_SendData(DEBUG_USART, (uint8_t) ch);
//		
//		/* �ȴ�������� */
//		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
//	
//		return (ch);
//}

////�ض���c�⺯��scanf������1����д����ʹ��scanf��getchar�Ⱥ���
//int fgetc(FILE *f)
//{
//		/* �ȴ������������� */
//		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(DEBUG_USART);
//}
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
