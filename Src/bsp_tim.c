#include "bsp_tim.h"
#include "BSPconfig.h"
#include "stm32f4xx_hal.h"

void UserTim1Config(void)//Ä¦²ÁÂÖ
{
	TIM1->CCER |=TIM_CCER_CC1E ;
	TIM1->CCER |=TIM_CCER_CC2E ;
	TIM1->CCER |=TIM_CCER_CC3E ;
	TIM1->CCER |=TIM_CCER_CC4E ;
	
	LL_TIM_EnableARRPreload(TIM1);
	LL_TIM_EnableAllOutputs(TIM1);
	
	LL_TIM_EnableCounter(TIM1); 
}

void UserTim3Config(void)
{
	LL_TIM_EnableIT_UPDATE(TIM3);
	LL_TIM_EnableCounter(TIM3);
}

void UserTim8Config(void)
{
	TIM8->CCER |=TIM_CCER_CC3E ;
	TIM8->CCER |=TIM_CCER_CC4E ;

	LL_TIM_EnableARRPreload(TIM8);
	LL_TIM_EnableAllOutputs(TIM8);

	LL_TIM_EnableCounter(TIM8);

}

