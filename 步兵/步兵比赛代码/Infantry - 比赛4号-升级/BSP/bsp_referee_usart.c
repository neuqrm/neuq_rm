
#include "bsp_referee_usart.h"
#include "referee.h"

/* ----------------------- Internal Data ----------------------------------- */

uint8_t Referee_Buffer0[REFEREE_BUFFER_LENGTH]; //double sbus rx buffer to save data
uint8_t Referee_Buffer1[REFEREE_BUFFER_LENGTH]; //double sbus rx buffer to save data
volatile uint16_t Rereree_Datalen = 0;										//保存数据长度数组的有效数据量
volatile uint16_t Rereree_Data_Bufferlen = 0;							//当前数据缓存堆栈的栈顶
uint16_t 	Referee_Data_Length[REFEREE_DATA_LENTH] = {0};				//保存存入数据缓存堆栈的数据段长度 数组
uint8_t 	Referee_Data_Buffer[REFEREE_BUFFER_ALLLENGTH] = {0};	//相当于一个数据栈 用来暂存来不及处理的数据

/* ----------------------- Function Implements ---------------------------- */
/******************************************************************************
* @fn RC_Init
*
* @brief configure stm32 usart1 port
* - USART Parameters
* - 100Kbps
* - 8-N-1
* - DMA Mode
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port it
* to the other platform.
*/
void Referee_USART_Config(void)
{
/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(REFEREE_USART_GPIO_CLK | REFEREE_USART_DMA_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(REFEREE_USART_CLK, ENABLE);
	GPIO_PinAFConfig(REFEREE_USART_GPIO_PORT,REFEREE_USART_GPIO_SOURCE, REFEREE_USART_GPIO_AF);
/* -------------- Configure GPIO ---------------------------------------*/
	{
		GPIO_InitTypeDef 	GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		GPIO_InitStructure.GPIO_Pin = REFEREE_USART_GPIO_PIN ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(REFEREE_USART_GPIO_PORT, &GPIO_InitStructure);
		
		USART_DeInit(REFEREE_USART);
		USART_InitStructure.USART_BaudRate = REFEREE_USART_BAUDRATE;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(REFEREE_USART,&USART_InitStructure);
		USART_DMACmd(REFEREE_USART,USART_DMAReq_Rx,ENABLE);
		USART_DMACmd(REFEREE_USART,USART_DMAReq_Tx,ENABLE);
		USART_ITConfig(REFEREE_USART, USART_IT_IDLE, ENABLE); //usart rx idle interrupt enabled
		//USART_ITConfig(REFEREE_USART, USART_IT_RXNE, ENABLE);		//usart rx idle interrupt enabled
		USART_Cmd(REFEREE_USART,ENABLE);
	}
/* -------------- Configure NVIC ---------------------------------------*/
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = REFEREE_USART_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
//	{
//		NVIC_InitTypeDef NVIC_InitStructure;
//		NVIC_InitStructure.NVIC_IRQChannel = REFEREE_DMA_IRQ;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//	}
/* -------------- Configure DMA -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(REFEREE_USART_DMA_STREAM);
		DMA_InitStructure.DMA_Channel = REFEREE_USART_DMA_CHANNEL;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&REFEREE_USART_DR_BASE;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Referee_Buffer0;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = REFEREE_BUFFER_SIZE;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Circular DMA_Mode_Normal
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init(REFEREE_USART_DMA_STREAM,&DMA_InitStructure);
//	DMA_ITConfig(REFEREE_USART_DMA_STREAM,DMA_IT_TC,ENABLE);
		DMA_DoubleBufferModeConfig(REFEREE_USART_DMA_STREAM,(uint32_t)&Referee_Buffer1,DMA_Memory_0);//DMA_Memory_0?????
		DMA_DoubleBufferModeCmd(REFEREE_USART_DMA_STREAM,ENABLE);
		DMA_Init(REFEREE_USART_DMA_STREAM,&DMA_InitStructure);
		DMA_Cmd(REFEREE_USART_DMA_STREAM,ENABLE);
	}
}



/******************************************************************************
* @fn RefereeDataProcess
*
* @brief 向数据缓冲数组中添加数据
* @pData a point to referee receive buffer.
* @return None.
* @note .
*/
void RefereeDataProcess(vu8 *pData,uint16_t Data_Length)
{
	uint8_t i;
	uint8_t Temp_Data[Data_Length];
	if (pData[0] == HEADSOF)
	{
			for (i=0;i<Data_Length;i++)
		{
			Temp_Data[i] = pData[i];
		}
		Referee_Data_Solve(Temp_Data);
	}
	
}


/******************************************************************************
* @fn Referee_Referee_DataBuffer_INC_INC
*
* @brief 向裁判系统数据队列中添加数据，加在队尾
* @pData 数据数组指针 数据数组长度.
* @return 1 数据添加成功 0 链表超长，添加失败
* @note RC_CtrlData is a global variable.you can deal with it in other place.
*/
uint8_t  Referee_DataBuffer_INC(uint8_t *point,uint16_t length)
{
	uint16_t i,j;//如果数据区空闲内存足够
	j = Rereree_Data_Bufferlen;
	if(Rereree_Datalen<(REFEREE_DATA_LENTH-1) && (Rereree_Data_Bufferlen+length)<REFEREE_BUFFER_ALLLENGTH)
	{
		for(i=0;i<length;i++)
		{Referee_Data_Buffer[i+j] = point[i];}
		Rereree_Data_Bufferlen = j+length;
		Rereree_Datalen++;
		Referee_Data_Length[Rereree_Datalen] = length;
		return 1;
	}
	else
	{return 0;}//数据堆栈区已满 无法保存数据
}

/******************************************************************************
* @fn Referee_Referee_DataBuffer_INC_DEC
*
* @brief 裁判系统数据栈 从栈顶取出数据
* @pData 待取数组指针 *pData 数据缓存区长度buflen 数据段长度保存数组位号 datalen
* @return None.
* @note you can deal with it in other place. pData数组长度与datalen一致
*/
void Referee_DataBuffer_DEC(uint8_t *pData,uint16_t buflen,uint16_t datalen)
{
	uint8_t i,j;
	j = buflen-datalen;
	for (i=0;i<datalen;i++)
	{pData[i] = Referee_Data_Buffer[j+i];}
	Rereree_Datalen--;
	Rereree_Data_Bufferlen = j;

}
/******************************************************************************
* @fn REFEREE_USART_IRQHandler
*
* @brief USART1 irq, we are care of ilde interrupt that means receiving the
one frame datas is finished.
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port
it
* to the other platform.
*/
uint8_t Referee_Buffers1[200][10] = {0};
//uint16_t DataLength[300]={0};
//uint8_t DataLengthSEQ[300]={0};
void REFEREE_USART_IRQHandler(void)
{
	if(USART_GetITStatus(REFEREE_USART, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(REFEREE_USART,USART_IT_RXNE);
		USART_ITConfig(REFEREE_USART, USART_IT_RXNE, DISABLE); //usart rx idle interrupt enabled
	}
	if(USART_GetITStatus(REFEREE_USART, USART_IT_IDLE) != RESET)
	{
		static uint16_t Realtime_RX_Length = 0;
		static uint16_t count=0,counts=0;
		uint8_t i=0;
	 //clear the idle pending flag
		(void)USART6->SR;
		(void)USART6->DR;
		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(REFEREE_USART_DMA_STREAM) == 0)
		{
			DMA_Cmd(REFEREE_USART_DMA_STREAM, DISABLE);
			Realtime_RX_Length = REFEREE_BUFFER_LENGTH - DMA_GetCurrDataCounter(REFEREE_USART_DMA_STREAM);
			REFEREE_USART_DMA_STREAM->NDTR = (uint16_t)REFEREE_BUFFER_LENGTH; //relocate the dma memory pointer to the beginning position
			REFEREE_USART_DMA_STREAM->CR |= (uint32_t)(DMA_SxCR_CT); //enable the current selected memory is Memory 1
			DMA_SetCurrDataCounter(REFEREE_USART_DMA_STREAM,(uint16_t)REFEREE_BUFFER_LENGTH);
			DMA_Cmd(REFEREE_USART_DMA_STREAM, ENABLE);
			RefereeDataProcess(Referee_Buffer0,Realtime_RX_Length);	
			memset(Referee_Buffer0, 0,sizeof(Referee_Buffer0));
//			DataLengthSEQ[counts++] = Referee_Buffer0[3];
//			DataLength[count++]=Realtime_RX_Length;
//			for (i=0;i<5;i++)
//			{
//				Referee_Buffers1[count++][i] = Referee_Buffer[1][i];
//			}
		}
		//Target is Memory1
		else
		{
			DMA_Cmd(REFEREE_USART_DMA_STREAM, DISABLE);
			Realtime_RX_Length = REFEREE_BUFFER_LENGTH - DMA_GetCurrDataCounter(REFEREE_USART_DMA_STREAM);
			REFEREE_USART_DMA_STREAM->NDTR = (uint16_t)REFEREE_BUFFER_LENGTH; //relocate the dma memory pointer to the beginning position
			REFEREE_USART_DMA_STREAM->CR &= ~(uint32_t)(DMA_SxCR_CT); //enable the current selected memory is Memory 0
			DMA_SetCurrDataCounter(REFEREE_USART_DMA_STREAM,(uint16_t)REFEREE_BUFFER_LENGTH);
			DMA_Cmd(REFEREE_USART_DMA_STREAM, ENABLE);
			RefereeDataProcess(Referee_Buffer1,Realtime_RX_Length);
			memset(Referee_Buffer1, 0,sizeof(Referee_Buffer1));
//			DataLengthSEQ[counts++] = Referee_Buffer1[3];
//			DataLength[count++]=Realtime_RX_Length;
//			for (i=0;i<5;i++)
//			{
//				Referee_Buffers1[count++][i] = Referee_Buffer[0][i];
//			}
		}
		if (count==299)
			count=0;
		if (counts==299)
			counts=0;
	}
}

void REFEREE_SendByte(uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(REFEREE_USART,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(REFEREE_USART, USART_FLAG_TXE) == RESET);
}
