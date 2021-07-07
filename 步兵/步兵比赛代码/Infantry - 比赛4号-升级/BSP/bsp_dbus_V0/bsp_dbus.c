
#include "bsp_dbus.h"


/* ----------------------- Internal Data ----------------------------------- */
#define RC_FRAME_LENGTH 						18
volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH]; //double sbus rx buffer to save data
//static RC_Ctl_t RC_Ctl;
//static DJi_RC rc;
RC_Ctl_t RC_Ctl;
DJi_RC rc;
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
void Dji_Remote_Init(void)
{
/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(DBUS_USART_GPIO_CLK | DBUS_USART_DMA_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(DBUS_USART_CLK, ENABLE);
	GPIO_PinAFConfig(DBUS_USART_GPIO_PORT,DBUS_USART_GPIO_SOURCE, DBUS_USART_GPIO_AF);
/* -------------- Configure GPIO ---------------------------------------*/
	{
		GPIO_InitTypeDef 	GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		GPIO_InitStructure.GPIO_Pin = DBUS_USART_GPIO_PIN ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(DBUS_USART_GPIO_PORT, &GPIO_InitStructure);
		
		USART_DeInit(DBUS_USART);
		USART_InitStructure.USART_BaudRate = DBUS_USART_BAUDRATE;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_Mode = USART_Mode_Rx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(DBUS_USART,&USART_InitStructure);
		USART_Cmd(DBUS_USART,ENABLE);
		USART_DMACmd(DBUS_USART,USART_DMAReq_Rx,ENABLE);
	}
/* -------------- Configure NVIC ---------------------------------------*/
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = DBUS_DMA_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
/* -------------- Configure DMA -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DBUS_USART_DMA_STREAM);
		DMA_InitStructure.DMA_Channel = DBUS_USART_DMA_CHANNEL;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&DBUS_USART_DR_BASE;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = DBUS_BUFFER_SIZE;
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
		DMA_Init(DBUS_USART_DMA_STREAM,&DMA_InitStructure);
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); //usart rx idle interrupt enabled
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //usart rx idle interrupt enabled
	//	DMA_ITConfig(DBUS_USART_DMA_STREAM,DMA_IT_TC,ENABLE);
		DMA_Cmd(DBUS_USART_DMA_STREAM,ENABLE);
	}
}



/******************************************************************************
* @fn RemoteDataProcess
*
 * @brief resolution rc protocol data.
* @pData a point to rc receive buffer.
* @return None.
* @note RC_CtrlData is a global variable.you can deal with it in other place.
*/
void RemoteDataProcess(vu8 *pData)
{
	if(pData == 0)
	{
		return;
	}
	RC_Ctl.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	RC_Ctl.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))& 0x07FF;
	RC_Ctl.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	RC_Ctl.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &0x07FF;
	RC_Ctl.rc.s1 = (((int16_t)pData[5] >> 4) & 0x000C) >> 2;
	RC_Ctl.rc.s2 = (((int16_t)pData[5] >> 4) & 0x0003);
	RC_Ctl.mouse.x = (int16_t)pData[6] | ((int16_t)pData[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = (int16_t)pData[8] | ((int16_t)pData[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = (int16_t)pData[10] | ((int16_t)pData[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = (int16_t)pData[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = (int16_t)pData[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v = (int16_t)pData[14] | ((int16_t)pData[15] << 8); //!< KeyBoard value
}

/******************************************************************************
* @fn DBUS_USART_IRQHandler
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

void DBUS_USART_IRQHandler(void)
{
	if(USART_GetITStatus(DBUS_USART, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(DBUS_USART,USART_IT_RXNE);
		USART_ITConfig(DBUS_USART, USART_IT_RXNE, DISABLE); //usart rx idle interrupt enabled
	}
	if(USART_GetITStatus(DBUS_USART, USART_IT_IDLE) != RESET)
	{
	 //clear the idle pending flag
		(void)USART1->SR;
		(void)USART1->DR;
		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream5) == 0)
		{
			DMA_Cmd(DBUS_USART_DMA_STREAM, DISABLE);
			DMA2_Stream5->NDTR = (uint16_t)RC_FRAME_LENGTH; //relocate the dma memory pointer to the beginning position
			DMA2_Stream5->CR |= (uint32_t)(DMA_SxCR_CT); //enable the current selected memory is Memory 1
			DMA_Cmd(DBUS_USART_DMA_STREAM, ENABLE);
			RemoteDataProcess(sbus_rx_buffer[1]);
			
		}
		//Target is Memory1
		else
		{
			DMA_Cmd(DBUS_USART_DMA_STREAM, DISABLE);
			DMA2_Stream5->NDTR = (uint16_t)RC_FRAME_LENGTH; //relocate the dma memory pointer to the beginning position
			DMA2_Stream5->CR &= ~(uint32_t)(DMA_SxCR_CT); //enable the current selected memory is Memory 0
			DMA_Cmd(DBUS_USART_DMA_STREAM, ENABLE);
			RemoteDataProcess(sbus_rx_buffer[0]);
			
		}
	}
}

