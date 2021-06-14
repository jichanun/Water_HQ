#include "task_chassis.h"
#include "driver_chassis.h"


ChassisSpeedMessegePort ChassisSpeed;

void ChassisSetSpeed(float SpeedX,float SpeedY,float Spin,float Error)
{
	ChassisSpeed.SetSpeedX	=	 -SpeedX/4;
	ChassisSpeed.SetSpeedY	=	 SpeedY/2;
	ChassisSpeed.Spin	=	 Spin/2;
	ChassisSpeed.SpeedError=Error/2;
}

void ChassisControlTask()
{
//	ChassisFollowCalculate(&ChassisSpeed);

//	ChassisShakeCalculate(&ChassisSpeed);
//	ChassisChangeFollow(&ChassisSpeed);
	
	//ChassisControl(ChassisSpeed);
}
int PWMNum=MIDDLE_PWM;
void Chassis_Init(void)
{
	LL_TIM_CC_EnableChannel(TIM8,LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM8,LL_TIM_CHANNEL_CH4);	
	LL_TIM_EnableCounter(TIM8);
	LL_TIM_EnableAllOutputs(TIM8);
	
	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH2);	
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_EnableAllOutputs(TIM2);
	//PWMNum=0;
	LL_TIM_OC_SetCompareCH1(TIM2,PWMNum);
	LL_TIM_OC_SetCompareCH2(TIM2,PWMNum);
	LL_TIM_OC_SetCompareCH3(TIM8,PWMNum);
	LL_TIM_OC_SetCompareCH4(TIM8,PWMNum);
			LL_TIM_OC_SetCompareCH1(TIM12,PWMNum);
		LL_TIM_OC_SetCompareCH2(TIM12,PWMNum);
	//HAL_Delay(1500);

}
//420-630-840
void Chassis_Control(void)
{
	#if  0  ///*********************调试接口
	LL_TIM_OC_SetCompareCH1(TIM2,PWMNum);
	LL_TIM_OC_SetCompareCH2(TIM2,PWMNum);
	LL_TIM_OC_SetCompareCH3(TIM8,PWMNum);
	LL_TIM_OC_SetCompareCH4(TIM8,PWMNum);
			LL_TIM_OC_SetCompareCH1(TIM12,PWMNum);
		LL_TIM_OC_SetCompareCH2(TIM12,PWMNum);

	#else
	PositionCaculate();	
	ChassisControl_PWM(ChassisSpeed);
	
	#endif

}