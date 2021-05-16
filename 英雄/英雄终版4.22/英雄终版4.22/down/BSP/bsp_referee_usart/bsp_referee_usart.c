
#include "bsp_referee_usart.h"
#include "referee.h"

/* ----------------------- Internal Data ----------------------------------- */

volatile unsigned char Referee_Buffer[2][REFEREE_BUFFER_LENGTH]; //double sbus rx buffer to save data
volatile uint16_t Rereree_Datalen = 0;										//??????????????????§¹??????
volatile uint16_t Rereree_Data_Bufferlen = 0;							//?????????????????
uint16_t 	Referee_Data_Length[REFEREE_DATA_LENTH] = {0};				//??????????????????????¦Ã??? ????
uint8_t 	Referee_Data_Buffer[REFEREE_BUFFER_ALLLENGTH] = {0};	//???????????? ???????????????????????

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
		USART_InitStructure.USART_Mode = USART_Mode_Rx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(REFEREE_USART,&USART_InitStructure);
		USART_Cmd(REFEREE_USART,ENABLE);
		USART_DMACmd(REFEREE_USART,USART_DMAReq_Rx,ENABLE);
	}
/* -------------- Configure NVIC ---------------------------------------*/
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = REFEREE_USART_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = REFEREE_DMA_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
/* -------------- Configure DMA -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(REFEREE_USART_DMA_STREAM);
		DMA_InitStructure.DMA_Channel = REFEREE_USART_DMA_CHANNEL;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&REFEREE_USART_DR_BASE;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Referee_Buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = REFEREE_BUFFER_SIZE;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(REFEREE_USART_DMA_STREAM,&DMA_InitStructure);
		USART_ITConfig(REFEREE_USART, USART_IT_IDLE, ENABLE); //usart rx idle interrupt enabled
		USART_ITConfig(REFEREE_USART, USART_IT_RXNE, ENABLE); //usart rx idle interrupt enabled
	//	DMA_ITConfig(REFEREE_USART_DMA_STREAM,DMA_IT_TC,ENABLE);
		DMA_Cmd(REFEREE_USART_DMA_STREAM,ENABLE);
	}
}



/******************************************************************************
* @fn RefereeDataProcess
*
* @brief ???????????????????????
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
* @brief ??????????????????????????????¦Â
* @pData ??????????? ???????øA??.
* @return 1 ?????????? 0 ?????????????????
* @note RC_CtrlData is a global variable.you can deal with it in other place.
*/
uint8_t  Referee_DataBuffer_INC(uint8_t *point,uint16_t length)
{
	uint16_t i,j;//??????????????????
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
	{return 0;}//???????????? ???????????
}

/******************************************************************************
* @fn Referee_Referee_DataBuffer_INC_DEC
*
* @brief ??????????? ????????????
* @pData ?????????? *pData ?????????????buflen ????¦Ã??????????¦Ë?? datalen
* @return None.
* @note you can deal with it in other place. pData???øA????datalen???
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
			DMA_Cmd(REFEREE_USART_DMA_STREAM, ENABLE);
			RefereeDataProcess(Referee_Buffer[1],Realtime_RX_Length);	
		}
		//Target is Memory1
		else
		{
			DMA_Cmd(REFEREE_USART_DMA_STREAM, DISABLE);
			Realtime_RX_Length = REFEREE_BUFFER_LENGTH - DMA_GetCurrDataCounter(REFEREE_USART_DMA_STREAM);
			REFEREE_USART_DMA_STREAM->NDTR = (uint16_t)REFEREE_BUFFER_LENGTH; //relocate the dma memory pointer to the beginning position
			REFEREE_USART_DMA_STREAM->CR &= ~(uint32_t)(DMA_SxCR_CT); //enable the current selected memory is Memory 0
			DMA_Cmd(REFEREE_USART_DMA_STREAM, ENABLE);
			RefereeDataProcess(Referee_Buffer[0],Realtime_RX_Length);
		}
	}
}

