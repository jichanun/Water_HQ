#ifndef _USART_BSP_H
#define _USART_BSP_H
#include "sys.h"
void USART6ConfigEnable(void);
void USART3ConfigEnable(void);
void USART1ConfigEnable(void);
void ConfigUsart1DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);
void ConfigUsart6DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);
void ConfigUsart3DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);

//WifiChange Begin
void WIFI_USARTConfigEnable(void);
void WIFI_ConfigUsartDMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);
//WifiChange End
#endif