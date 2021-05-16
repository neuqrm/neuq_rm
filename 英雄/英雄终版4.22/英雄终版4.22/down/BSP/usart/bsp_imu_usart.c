#include "bsp_imu_usart.h"

void bsp_imu_usart_init(void)  //����8
	{
 
	//�����ж����ȷ���
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//����һ��GPIO�ṹ�����
	GPIO_InitTypeDef GPIO_InitStructure;
	//����һ��USART�ṹ�����
	USART_InitTypeDef USART_InitStructure;
	//ʹ��USART1����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);
	//ʹ��GPIO����ʱ��	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	//����GPIO�ṹ����������ø�IO����Ϊ����
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 |GPIO_Pin_1;//����IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;         //����ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;    //50MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;	   //�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;		   //����
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	//����USART�ṹ�����
	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//Ӳ��������   
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; //�շ�ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;    //��У׼
	USART_InitStructure.USART_StopBits=USART_StopBits_1; //1λֹͣλ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;  //�ֳ�Ϊ8
	//��ȷIOΪ�ι��ܽ�������
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_UART8); //
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_UART8);
	
	//���ڽ��г�ʼ��
	USART_Init(UART8,&USART_InitStructure);
	//���ô��ڽ����ж�
	USART_ITConfig(UART8,USART_IT_RXNE,ENABLE);
    //�����ж�������ṹ���������
	NVIC_InitTypeDef NVIC_InitStructure_usart;
	//����ô����ж����������
	NVIC_InitStructure_usart.NVIC_IRQChannel=UART8_IRQn;
	NVIC_InitStructure_usart.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure_usart.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�����
	NVIC_InitStructure_usart.NVIC_IRQChannelSubPriority=2;		//��Ӧ���ȼ�����
	//��ʼ�������ȼ�����
	NVIC_Init(&NVIC_InitStructure_usart);
	//ʹ�ܴ���1
	USART_Cmd(UART8,ENABLE);     
}
