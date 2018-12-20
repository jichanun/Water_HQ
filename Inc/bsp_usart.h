#ifndef _USART_BSP_H
#define _USART_BSP_H
#include "sys.h"

void USART1ConfigEnable(void);
void ConfigUsart1DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);

#endif