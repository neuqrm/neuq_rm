/**
  ******************************************************************************
  * @file    bsp_IMU_usart.c
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
  
#include "bsp_imu_usart.h"
#include "imu.h"


 /**
  * @brief  IMU_USART GPIO ����,����ģʽ���á�115200 8-N-1 ���жϽ���ģʽ
  * @param  ��
  * @retval ��
  */
void IMU_USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
  RCC_AHB1PeriphClockCmd(IMU_USART_RX_GPIO_CLK|IMU_USART_TX_GPIO_CLK,ENABLE);

  /* ʹ�� USART ʱ�� */
  IMU_USART_PeriphClockCmd(IMU_USART_CLK, ENABLE);
  
  /* GPIO��ʼ�� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = IMU_USART_TX_PIN  ;  
  GPIO_Init(IMU_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = IMU_USART_RX_PIN;
  GPIO_Init(IMU_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* ���� PXx �� USARTx_Tx*/
  GPIO_PinAFConfig(IMU_USART_RX_GPIO_PORT,IMU_USART_RX_SOURCE,IMU_USART_RX_AF);

  /*  ���� PXx �� USARTx__Rx*/
  GPIO_PinAFConfig(IMU_USART_TX_GPIO_PORT,IMU_USART_TX_SOURCE,IMU_USART_TX_AF);
  
  /* ���ô�IMU_USART ģʽ */
  /* ���������ã�IMU_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = IMU_USART_BAUDRATE;
  /* �ֳ�(����λ+У��λ)��8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* ֹͣλ��1��ֹͣλ */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* У��λѡ�񣺲�ʹ��У�� */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* Ӳ�������ƣ���ʹ��Ӳ���� */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* ���USART��ʼ������ */
  USART_Init(IMU_USART, &USART_InitStructure); 

  /* ʹ�ܴ��� */
  USART_Cmd(IMU_USART, ENABLE);
	
	USART_ITConfig(IMU_USART, USART_IT_RXNE, ENABLE);//��������ж�
	
		//Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = IMU_USART_IRQ ;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}




/*****************  ����һ���ַ� **********************/
void IMU_SendByte(uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(IMU_USART,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(IMU_USART, USART_FLAG_TXE) == RESET);
}

/*****************  �����ַ��� **********************/
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
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(IMU_USART,USART_FLAG_TC)==RESET)
  {}
}

void IMU_USART_IRQHandler(void)
{
	uint8_t Temp = 0;
	if(USART_GetITStatus(IMU_USART,USART_IT_RXNE)!=RESET)
	{		
		Temp = USART_ReceiveData( IMU_USART );;	//���յ������ݴ��뻺������
		IMU_Data_Solve(Temp);
	}
	
}
/*********************************************END OF FILE**********************/
