#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"
#include "driver_remote.h"
//遥控器灵敏度
#define YAW_REMOTE_SENSITIVE (0.0005f)
#define PITCH_REMOTE_SENSITIVE (0.0005f)

GimbalSetLocationStruct	GimbalSetLocationDataTemp;
extern  RemoteDataUnion RemoteData;
extern  FeedMotorStruct FeedMotor;
extern  SwitchStruct Switch;
extern  int  Target_mode;

//换弹电机控制标志位
int flag_init=0; //换弹电机初始化标志位
//Switch.Reload1 换弹标志位
u8 GimbalInitFlag = 2;//云台初始化标识位
 float yaw_OFFSET;


void GimbalControlTask()
{
	//Step1:Get Gyro and encoder value
	GyroAndEncoderDataGet();
	
	if(GimbalInitFlag)
	{
		GimbalInitFlag--;
		GimbalSetLocationDataTemp.PitchSetLocation	=	PITCH_INIT_VALUE;
		GimbalSetLocationDataTemp.YawSetLocation	=	YAW_INIT_VALUE;
		GimbalSetLocationDataTemp.FlagPitchUseEncoder	=	0;
		GimbalSetLocationDataTemp.FlagYawUseEncoder	=	0;
		yaw_OFFSET =Gyroscope.yaw/360;
		GimbalSetLocationDataTemp.YawSetLocation	=	yaw_OFFSET;//使用陀螺仪则取消注释

	}
///**************************换弹电机（2006）**********************************/
//	//初始化
//	if(flag_init==0)
//	{
//	  FeedMotor.Location.SetLocation=-9;
//		if(Switch.Init==1)
//		{
//			FeedMotor.Location.SetLocation=FeedMotor.Location.Location;
//			flag_init=1;
//		}
//	}
//	
//	if(Switch.Reload1==1)
//	{
//		FeedMotor.Location.SetLocation=FeedMotor.Location.Location+9;
//		Switch.Reload1=0;
//	}
//	
	//FeedMotorDataUpdate();
	//MotorLocationControlLogic();

	//Gimbal
	GimbalDataInput(GimbalSetLocationDataTemp);
	GimbalSpeedDataUpdate();									//云台速度更新（使用陀螺仪速度）
	GimbalControlCalculateAndSend();
	

}

void YawSetLocationValueChange(float Yaw)
{
	
	GimbalSetLocationDataTemp.YawSetLocation	+=	Yaw;
}

void PitchSetLocationValueChange(float Pitch)
{
	GimbalSetLocationDataTemp.PitchSetLocation	+=	Pitch;
}

void StraightLineMotorInit(void)
{
	LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM4);
	LL_TIM_EnableAllOutputs(TIM4);
	LL_TIM_OC_SetCompareCH2(TIM4,10000);
}
void StraightLineMotorControl(void)
{
	//控制两个输出IO口的开闭
//   //前哨战
//   	if(Target_mode==0)
//	{
//		 LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_15);
//		 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
//	   if(Switch.Target0==1)
//	   {
//		  LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
//		  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
//     }		
//	}
// 
//   //基地  
//  	if(Target_mode==1)
//	{
//	 	 LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_3);
//		 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
//	   if(Switch.Target1==1)
//	   {
//		  LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
//		  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
//	   }		
//	}	
if((RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)>10)
		{
		 LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_15);
		 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
		}
    if((RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)<-10)
		{
	 	 LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_3);
		 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
		}	
		if((RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)>=-10&&(RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)<=10)
		{
		  LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
		  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
		}
		
}
