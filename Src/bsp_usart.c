#include "bsp_usart.h"
#include "sys.h"

u32 Usart1DMAMemoryBaseAddress,Usart1DMABufferSize;
void ConfigUsart1DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)
{
	Usart1DMAMemoryBaseAddress=DMA_Memory0BaseAddr;
	Usart1DMABufferSize=DMA_BufferSize;
	
	USART1ConfigEnable();
}

void USART1ConfigEnable(void)
{
	LL_DMA_SetMemoryAddress(DMA2,LL_DMA_STREAM_5,(u32)Usart1DMAMemoryBaseAddress);
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_5,(u32)&USART1->DR);
	LL_DMA_SetDataLength(DMA2,LL_DMA_STREAM_5,Usart1DMABufferSize);	
	
	LL_USART_EnableIT_IDLE(USART1);
  LL_USART_EnableDMAReq_RX(USART1);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
}
