#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"

//遥控器灵敏度
#define YAW_REMOTE_SENSITIVE (0.0005f)
#define PITCH_REMOTE_SENSITIVE (0.0005f)

u8 Flag = 1;

GimbalSetLocationStruct	GimbalSetLocationDataTemp;

void GimbalControlTask()
{
	//Step1:Get Gyro and encoder value
	GyroAndEncoderDataGet();
	
	if(Flag)
	{
		Flag=0;
		GimbalSetLocationDataTemp.PitchSetLocation	=	PITCH_INIT_VALUE;
		GimbalSetLocationDataTemp.YawSetLocation	=	YAW_INIT_VALUE;
		GimbalSetLocationDataTemp.FlagPitchUseEncoder	=	1;
		GimbalSetLocationDataTemp.FlagYawUseEncoder	=	1;
	}
	
	//FeedMotor
	FeedMotorDataUpdate();
	MotorLocationControlLogic();
	
	//Gimbal
	GimbalDataInput(GimbalSetLocationDataTemp);
	GimbalSpeedDataUpdate();									//云台速度更新
//	GimbalControlCalculateAndSend();
	

}

void YawSetLocationValueChange(float Yaw)
{
	
	GimbalSetLocationDataTemp.YawSetLocation	=	Yaw;
}

void PitchSetLocationValueChange(float Pitch)
{
	GimbalSetLocationDataTemp.PitchSetLocation	=	Pitch;
}

