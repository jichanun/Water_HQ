#include "bsp_usart.h"
#include "sys.h"
#include "task_wifi.h"

u32 Usart1DMAMemoryBaseAddress,Usart1DMABufferSize;
u32 Usart2DMAMemoryBaseAddress,Usart2DMABufferSize;

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

u32 Usart6DMAMemoryBaseAddress,Usart6DMABufferSize;
void ConfigUsart6DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)
{
	Usart6DMAMemoryBaseAddress=DMA_Memory0BaseAddr;
	Usart6DMABufferSize=DMA_BufferSize;
	
	USART6ConfigEnable();
}

void USART6ConfigEnable(void)
{
	LL_DMA_SetMemoryAddress(DMA2,LL_DMA_STREAM_1,(u32)Usart6DMAMemoryBaseAddress);
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_1,(u32)&USART6->DR);
	LL_DMA_SetDataLength(DMA2,LL_DMA_STREAM_1,Usart6DMABufferSize);	
	
	LL_USART_EnableIT_IDLE(USART6);
  LL_USART_EnableDMAReq_RX(USART6);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}
void ConfigUsart2DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)
{
	Usart2DMAMemoryBaseAddress=DMA_Memory0BaseAddr;
	Usart2DMABufferSize=DMA_BufferSize;
	
	USART2ConfigEnable();
}

void USART2ConfigEnable(void)
{
	LL_DMA_SetMemoryAddress(DMA1,LL_DMA_STREAM_5,(u32)Usart2DMAMemoryBaseAddress);
	LL_DMA_SetPeriphAddress(DMA1,LL_DMA_STREAM_5,(u32)&USART2->DR);
	LL_DMA_SetDataLength(DMA1,LL_DMA_STREAM_5,Usart2DMABufferSize);	
	LL_USART_EnableIT_IDLE(USART2);
  LL_USART_EnableDMAReq_RX(USART2);
//	LL_USART_EnableDMAReq_TX(USART2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
//	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
}


//WifiChange Begin   注：如果需要更改wifi的串口号，只需在cube下重新生成相应配置，在此处更改dma和dma通道信息，并在main函数中wifiinit()第三个参数写相应串口
//注：wifi串口波特率115200
u32 WIFI_UsartDMAMemoryBaseAddress,WIFI_UsartDMABufferSize;
void WIFI_ConfigUsartDMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)//wifi串口初始化必须在WiFi初始化之后,否则会出现WIFI_USART为空的情况
{
	WIFI_UsartDMAMemoryBaseAddress=DMA_Memory0BaseAddr;
	WIFI_UsartDMABufferSize=DMA_BufferSize;
	
	WIFI_USARTConfigEnable();
}

void WIFI_USARTConfigEnable(void)//
{
	LL_DMA_SetMemoryAddress(DMA1,LL_DMA_STREAM_1,(u32)WIFI_UsartDMAMemoryBaseAddress);
	LL_DMA_SetPeriphAddress(DMA1,LL_DMA_STREAM_1,(u32)&WIFI_USART->DR);
	LL_DMA_SetDataLength(DMA1,LL_DMA_STREAM_1,WIFI_UsartDMABufferSize);	
	
	LL_USART_EnableIT_IDLE(WIFI_USART);//空闲中断
  LL_USART_EnableDMAReq_RX(WIFI_USART);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
}


//WifiChange End