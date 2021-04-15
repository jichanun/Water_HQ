#ifndef _TIM_BSP_H
#define _TIM_BSP_H
#include "sys.h"

void UserTim1Config(void);
void UserTim3Config(void);
void UserTim8Config(void);
void ConfigTIM3DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);
void TIM3ConfigEnable(void);

#endif