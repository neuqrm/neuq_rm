
#include "bsp_supercap_usart.h"
#include "referee.h"

 /**
  * @brief  CAP_USART GPIO ����,����ģʽ���á�115200 8-N-1 ���жϽ���ģʽ
  * @param  ��
  * @retval ��
  */
void CAP_USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
  RCC_AHB1PeriphClockCmd(CAP_USART_RX_GPIO_CLK|CAP_USART_TX_GPIO_CLK,ENABLE);

  /* ʹ�� USART ʱ�� */
  CAP_USART_PeriphClockCmd(CAP_USART_CLK, ENABLE);
  
  /* GPIO��ʼ�� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CAP_USART_TX_PIN  ;  
  GPIO_Init(CAP_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CAP_USART_RX_PIN;
  GPIO_Init(CAP_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* ���� PXx �� USARTx_Tx*/
  GPIO_PinAFConfig(CAP_USART_RX_GPIO_PORT,CAP_USART_RX_SOURCE,CAP_USART_RX_AF);

  /*  ���� PXx �� USARTx__Rx*/
  GPIO_PinAFConfig(CAP_USART_TX_GPIO_PORT,CAP_USART_TX_SOURCE,CAP_USART_TX_AF);
  
  /* ���ô�CAP_USART ģʽ */
  /* ���������ã�CAP_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = CAP_USART_BAUDRATE;
  /* �ֳ�(����λ+У��λ)��8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* ֹͣλ��1��ֹͣλ */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* У��λѡ�񣺲�ʹ��У�� */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* Ӳ�������ƣ���ʹ��Ӳ���� */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
  USART_InitStructure.USART_Mode = USART_Mode_Tx ;//| USART_Mode_Tx;
  /* ���USART��ʼ������ */
  USART_Init(CAP_USART, &USART_InitStructure); 

  /* ʹ�ܴ��� */
  USART_Cmd(CAP_USART, ENABLE);
	
	USART_ITConfig(CAP_USART, USART_IT_RXNE, ENABLE);//��������ж�
	
		//Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = CAP_USART_IRQ ;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}




/*****************  ����һ���ַ� **********************/
void CAP_SendByte(uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(CAP_USART,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(CAP_USART, USART_FLAG_TXE) == RESET);
}
  //�������ݵȼ�����  ��ʼΪ5
char cap_level='5';

void supercap_Init(void)
{
	CAP_USART_Config();                //��ʼ���������ݵĴ���
	CAP_SendByte(cap_level);             //��ʼ���������ݵĵȼ�
}

/********         ʹ�ó�������          *********/



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