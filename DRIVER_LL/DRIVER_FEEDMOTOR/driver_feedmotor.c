#include "driver_feedmotor.h"
#include "pid.h"
#include "math.h"
#include "bsp_can.h"

FeedMotorStruct FeedMotor={0};

#define FEEDMOTOR_SPEED_KP	(1.6f)
#define FEEDMOTOR_SPEED_KI	(0)
#define FEEDMOTOR_SPEED_KD	(0)

#define FEEDMOTOR_LOCATION_KP	(0.25f)
#define FEEDMOTOR_LOCATION_KI	(0)
#define FEEDMOTOR_LOCATION_KD	(0.0f)

void FeedMotorInit(void)
{
	FeedMotor.PIDSpeed.OutMax=1;
	FeedMotor.PIDSpeed.OutMin=-1;
	FeedMotor.PIDSpeed.calc=&PidCalc;
	FeedMotor.PIDSpeed.clear=&PidClear;
	FeedMotor.PIDSpeed.clear(&FeedMotor.PIDSpeed);
	
	FeedMotor.PIDLocation.OutMax=1;
	FeedMotor.PIDLocation.OutMin=-1;
	FeedMotor.PIDLocation.calc=&PidCalc;
	FeedMotor.PIDLocation.clear=&PidClear;
	FeedMotor.PIDLocation.clear(&FeedMotor.PIDLocation);
	
	FeedMotor.PIDSpeed.Kp=FEEDMOTOR_SPEED_KP;
	FeedMotor.PIDSpeed.Ki=FEEDMOTOR_SPEED_KI;
	FeedMotor.PIDSpeed.Kd=FEEDMOTOR_SPEED_KD;
	
	FeedMotor.PIDLocation.Kp=FEEDMOTOR_LOCATION_KP;
	FeedMotor.PIDLocation.Ki=FEEDMOTOR_LOCATION_KI;
	FeedMotor.PIDLocation.Kd=FEEDMOTOR_LOCATION_KD;
}


void FeedMotorSpeedDataUpdate(void)							
{
	FeedMotor.Speed.Speed=((int16_t)((FeedMotor.ReceiveMessege[2]<<8)|FeedMotor.ReceiveMessege[3]));
	FeedMotor.Speed.Speed = FeedMotor.Speed.Speed/9600;
}

void FeedMotorCurrentDataUpdate(void)		
{
	FeedMotor.Current = ((int16_t)((FeedMotor.ReceiveMessege[4]<<8)|FeedMotor.ReceiveMessege[5]));
}


float LocationDataLast=0;
int LocationCount=0;
void FeedMotorLocationDataUpdate(void)
{
	float LocationDataTemp;

	LocationDataTemp = ((int16_t)((FeedMotor.ReceiveMessege[0]<<8)|FeedMotor.ReceiveMessege[1]));
	LocationDataTemp = LocationDataTemp/8191;
	
	if			(LocationDataTemp	-	LocationDataLast	>	0.50f)
		LocationCount--;
	else if	(LocationDataTemp	-	LocationDataLast	<	-0.50f)
		LocationCount++;
	
	LocationDataLast 	= LocationDataTemp;
	
	FeedMotor.Location.Location	=	LocationDataTemp + LocationCount;
}

void FeedMotorLocationClear(void)
{
		LocationDataLast = 0;
		LocationCount = 0;
		FeedMotor.Location.Location = 0;
}

void FeedMotorDataUpdate()
{
		FeedMotorSpeedDataUpdate();
		FeedMotorLocationDataUpdate();
		FeedMotorCurrentDataUpdate();
}

void MotorLocationControlLogic(void)
{
//	u8 Can2FeedMotorSendMessege[8] = {0};
	
	FeedMotor.PIDLocation.Ref	=	FeedMotor.Location.SetLocation;
	FeedMotor.PIDLocation.Fdb	=	FeedMotor.Location.Location;
	
	FeedMotor.PIDLocation.calc(&FeedMotor.PIDLocation);
	
	FeedMotor.Speed.SetSpeed	=	FeedMotor.PIDLocation.Out;
	
	FeedMotor.PIDSpeed.Ref	=	FeedMotor.Speed.SetSpeed;
	FeedMotor.PIDSpeed.Fdb	=	FeedMotor.Speed.Speed;
	
	FeedMotor.PIDSpeed.calc(&FeedMotor.PIDSpeed);
	
//	Can2FeedMotorSendMessege[0]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))>>8;
//	Can2FeedMotorSendMessege[1]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))&0x00ff;

//#if DEBUG_USE_GIMBALMOTOR_CANSEND
//	CAN2_Send_Msg_FeedMotor(Can2FeedMotorSendMessege,8);
//#endif
}


void MotorSpeedControlLogic(void)
{
	FeedMotor.PIDSpeed.Ref	=	FeedMotor.Speed.SetSpeed;
	FeedMotor.PIDSpeed.Fdb	=	FeedMotor.Speed.Speed;
	
	FeedMotor.PIDSpeed.calc(&FeedMotor.PIDSpeed);
}

void SetFeedMotorSetSpeed(float SetSpeed)
{
	FeedMotor.Speed.SetSpeed = SetSpeed;
}

void SetFeedMotorSetLocation(float SetLocation)
{
	FeedMotor.Location.SetLocation = SetLocation;
}

void FeedMotorPIDClear(void)
{
	FeedMotor.PIDSpeed.clear(&FeedMotor.PIDSpeed);
	FeedMotor.PIDLocation.clear(&FeedMotor.PIDLocation);
}

float GetFeedMotorLocation(void)
{
	return FeedMotor.Location.Location;
}

float GetFeedMotorSpeed(void)
{
	return FeedMotor.Speed.Speed;
}

float GetFeedMotorLocationError(void)
{
		return FeedMotor.PIDLocation.Err;
}

float GetFeedMotorCurrent(void)
{
		return FeedMotor.Current;
}

