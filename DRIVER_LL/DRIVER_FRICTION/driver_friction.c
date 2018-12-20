#include "driver_friction.h"


void FrictionStop()
{
	LL_TIM_OC_SetCompareCH3(TIM1, 1000-1);
	LL_TIM_OC_SetCompareCH4(TIM1, 1000-1);	
}

void FrictionStart()
{
	LL_TIM_OC_SetCompareCH3(TIM1, 1250);
	LL_TIM_OC_SetCompareCH4(TIM1, 1250);
}

void FrictionControl(u8 Judge)
{
	if(Judge)
		FrictionStart();
	else
		FrictionStop();
}

