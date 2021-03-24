#include "can.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

	GPIO_InitTypeDef 				GPIO_InitStructure; 
	CAN_InitTypeDef        			CAN_InitStructure;
	CAN_FilterInitTypeDef  			CAN_FilterInitStructure;
	#if CAN1_RX0_INT_ENABLE 
		NVIC_InitTypeDef 			NVIC_InitStructure;
	#endif
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); 
	CAN_InitStructure.CAN_TTCM=DISABLE;	  
	CAN_InitStructure.CAN_ABOM=ENABLE;	  
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;	
	CAN_InitStructure.CAN_RFLM=DISABLE;	 
	CAN_InitStructure.CAN_TXFP=DISABLE;	
	CAN_InitStructure.CAN_Mode= mode;	 
	CAN_InitStructure.CAN_SJW=tsjw;	
	CAN_InitStructure.CAN_BS1=tbs1; 
	CAN_InitStructure.CAN_BS2=tbs2;
	CAN_InitStructure.CAN_Prescaler=brp; 
	CAN_Init(CAN1, &CAN_InitStructure);  
	CAN_FilterInitStructure.CAN_FilterNumber=0;	 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; 
	CAN_FilterInit(&CAN_FilterInitStructure);
	#if CAN1_RX0_INT_ENABLE
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		    
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	#endif
	return 0;
}   


u8 CAN1_Send_CHASSIS_Msg(u8* msg)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x200;	 
	TxMessage.ExtId=0;	 
	TxMessage.IDE=0;		 
	TxMessage.RTR=0;		
	TxMessage.DLC=8;							
	for(i=0;i<8;i++)
	TxMessage.Data[i]=msg[i];				
	mbox= CAN_Transmit(CAN1, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;
	if(i>=0XFFF)return 1;
	return 0;
}
u8 CAN1_Send_Trigger_Msg(u8* msg)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x1FF;	 
	TxMessage.ExtId=0;	
	TxMessage.IDE=0;		 
	TxMessage.RTR=0;		 
	TxMessage.DLC=8;							 
	for(i=0;i<8;i++)
	TxMessage.Data[i]=msg[i];				
	mbox= CAN_Transmit(CAN1, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	
	if(i>=0XFFF)return 1;
	return 0;
}


u8 CAN1_Send_GIMBAL_Msg(u8* msg)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x2FF;	 
	TxMessage.ExtId=0;	 
	TxMessage.IDE=0;		
	TxMessage.RTR=0;		
	TxMessage.DLC=8;							 
	for(i=0;i<8;i++)
	TxMessage.Data[i]=msg[i];				 
	mbox= CAN_Transmit(CAN1, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	
	if(i>=0XFFF)return 1;
	return 0;
}

u8 CAN1_Receive_Msg(u8 *buf)
{
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}







