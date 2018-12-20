/****************************************************
*			Title:		云台控制
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#include "driver_gimbal.h"
#include "driver_mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "bsp_can.h"
#include "math.h"
#include "driver_feedmotor.h"

#define ENCODER_LINE (8191)
#define GYROY_OFFSET (0)
#define GYROZ_OFFSET (0)

#define PITCH (1)

#define PITCH_LOCATION_KP (2.0f)				//2
#define PITCH_LOCATION_KI (0)
#define PITCH_LOCATION_KD (0)
#define PITCH_SPEED_KP (20)							//20
#define PITCH_SPEED_KI (0)
#define PITCH_SPEED_KD (0)

#define YAW (0)

#define YAW_LOCATION_KP (0.9f)					//1.2
#define YAW_LOCATION_KI (0)
#define YAW_LOCATION_KD (0)
#define YAW_LOCATION_KC (0)
#define YAW_SPEED_KP (25)						//35
#define YAW_SPEED_KI (0)
#define YAW_SPEED_KD (0)

GimbalMotorStruct	YawMotor,PitchMotor;

void GimbalInit(void)
{
	YawMotor.Location.SetLocation=YAW_INIT_VALUE;
	PitchMotor.Location.SetLocation=PITCH_INIT_VALUE;	
	//Yaw轴位置PID初始化
	YawMotor.PIDLocation.OutMax=1;
	YawMotor.PIDLocation.OutMin=-1;
	YawMotor.PIDLocation.calc=&PidCalc;
	YawMotor.PIDLocation.clear=&PidClear;
	YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
	
	YawMotor.PIDSpeed.OutMax=1;
	YawMotor.PIDSpeed.OutMin=-1;
	YawMotor.PIDSpeed.calc=&PidCalc;
	YawMotor.PIDSpeed.clear=&PidClear;
	YawMotor.PIDSpeed.clear(&YawMotor.PIDSpeed);
	
	PitchMotor.PIDLocation.OutMax=1;
	PitchMotor.PIDLocation.OutMin=-1;
	PitchMotor.PIDLocation.calc=&PidCalc;
	PitchMotor.PIDLocation.clear=&PidClear;
	PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
	
	PitchMotor.PIDSpeed.OutMax=1;
	PitchMotor.PIDSpeed.OutMin=-1;
	PitchMotor.PIDSpeed.calc=&PidCalc;
	PitchMotor.PIDSpeed.clear=&PidClear;
	PitchMotor.PIDSpeed.clear(&PitchMotor.PIDSpeed);
	
	YawMotor.PIDLocation.Kp	= YAW_LOCATION_KP;
	YawMotor.PIDLocation.Ki	= YAW_LOCATION_KI;
	YawMotor.PIDLocation.Kd	= YAW_LOCATION_KD;
	YawMotor.PIDLocation.Kc	= YAW_LOCATION_KC;
	
	YawMotor.PIDSpeed.Kp	=	YAW_SPEED_KP;
	YawMotor.PIDSpeed.Ki	=	YAW_SPEED_KI;
	YawMotor.PIDSpeed.Kd	=	YAW_SPEED_KD;
	
	PitchMotor.PIDLocation.Kp	= PITCH_LOCATION_KP;
	PitchMotor.PIDLocation.Ki	= PITCH_LOCATION_KI;
	PitchMotor.PIDLocation.Kd	= PITCH_LOCATION_KD;
	
	PitchMotor.PIDSpeed.Kp	=	PITCH_SPEED_KP;
	PitchMotor.PIDSpeed.Ki	=	PITCH_SPEED_KI;
	PitchMotor.PIDSpeed.Kd	=	PITCH_SPEED_KD;
}

void GimbalReturnToInitLocation(u8 IfPitch,u8 IfYaw)
{
	if(IfYaw)
		YawMotor.Location.SetLocation=YAW_INIT_VALUE;
	if(IfPitch)
		PitchMotor.Location.SetLocation=PITCH_INIT_VALUE;	
}

void GimbalPIDLocationClear(u8 IfPitch,u8 IfYaw)
{
	if(IfYaw)
		YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
	if(IfPitch)
		PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
}
void GimbalSpeedDataUpdate()									//云台速度更新
{
	short gyrox,gyroy,gyroz; //局部变量好像会出问题
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);

	YawMotor.Speed.Speed = (gyroz-GYROZ_OFFSET) / 32.8;
	YawMotor.Speed.Speed	=	YawMotor.Speed.Speed	/	2000;
	
	PitchMotor.Speed.Speed = (gyroy-GYROY_OFFSET) / 32.8;
	PitchMotor.Speed.Speed	=	PitchMotor.Speed.Speed	/	2000;
}

GyroDataStruct YawPitchGyroDataUpdate(float YawData,float PitchData)				//陀螺仪更新
{
	GyroDataStruct GyroData;
	float YawGyroDataTemp;
	static float YawGyroDataLast=0;
	static int YawGyroCount=0;
	
	YawGyroDataTemp	=	YawData;
	YawGyroDataTemp	=	YawGyroDataTemp	/	360.0f;
	
	if			(YawGyroDataTemp	-	YawGyroDataLast	>	0.8f)
		YawGyroCount--;
	else if	(YawGyroDataTemp	-	YawGyroDataLast	<	-0.8f)
		YawGyroCount++;
	
	YawGyroDataLast 	= YawGyroDataTemp ;
	GyroData.Yaw	=	YawGyroDataTemp + YawGyroCount;
	
	GyroData.Pitch	=	PitchData;
	GyroData.Pitch	=	GyroData.Pitch / 180.0f;
	return GyroData;
}

EncoderDataStruct YawPitchEncoderDataUpdate()										//编码器数据更新
{
	EncoderDataStruct EncoderData;
	static float YawEncoderDataTemp=YAW_TEMPSET,YawEncoderDataLast=YAW_TEMPSET;
	static int YawEncoderCount=0;
	
	static float PitchEncoderDataTemp=PITCH_TEMPSET,PitchEncoderDataLast=PITCH_TEMPSET;
	static int PitchEncoderCount=0;
	
	YawEncoderDataTemp	=	(short int)((YawMotor.CANReceiveMessege[0]<<8)|YawMotor.CANReceiveMessege[1]);
	YawEncoderDataTemp	=	YawEncoderDataTemp	/	ENCODER_LINE;
	
	if			(YawEncoderDataTemp	-	YawEncoderDataLast	>	0.8f)
		YawEncoderCount--;
	else if	(YawEncoderDataTemp	-	YawEncoderDataLast	<	-0.8f)
		YawEncoderCount++;
	
	EncoderData.Yaw	=	YawEncoderCount	+	YawEncoderDataTemp;
	YawEncoderDataLast	=	YawEncoderDataTemp;
	
	PitchEncoderDataTemp	=	(short int)((PitchMotor.CANReceiveMessege[0]<<8)|PitchMotor.CANReceiveMessege[1]);
	PitchEncoderDataTemp	=	PitchEncoderDataTemp	/	ENCODER_LINE;
	
	if			(PitchEncoderDataTemp	-	PitchEncoderDataLast	>	0.8f)
		PitchEncoderCount--;
	else if	(PitchEncoderDataTemp	-	PitchEncoderDataLast	<	-0.8f)
		PitchEncoderCount++;

	EncoderData.Pitch	=	PitchEncoderCount	+	PitchEncoderDataTemp;
	PitchEncoderDataLast	=	PitchEncoderDataTemp;
	
	return EncoderData;
}

GimbalSetLocationStruct GimbalSetLocationData;
float MPU9250Yaw;
float MPU9250Pitch;
GyroDataStruct	GyroDataSave;
EncoderDataStruct	EncoderDataSave;
float GetYawEncoderValue()
{
	return	EncoderDataSave.Yaw;
}
float GetYawGyroValue()
{
	return	GyroDataSave.Yaw;
}

void GyroAndEncoderDataGet(void)					//陀螺仪值定时更新，1ms
{
	float roll;
	if(mpu_mpl_get_data(&MPU9250Pitch,&roll,&MPU9250Yaw)==0)
	{
			GyroDataSave = YawPitchGyroDataUpdate(MPU9250Yaw,MPU9250Pitch);
	}
	EncoderDataSave	=	YawPitchEncoderDataUpdate();
}
void YawSetLocationValueChange(float Yaw);
void PitchSetLocationValueChange(float Pitch);
void GimbalDataInput(GimbalSetLocationStruct GimbalData)
{
	static u8 FlagPitchUseEncoderTemp=1,FlagYawUseEncoderTemp=1;
	GimbalSetLocationData.YawSetLocation	=	GimbalData.YawSetLocation;
	GimbalSetLocationData.PitchSetLocation=	GimbalData.PitchSetLocation;
//云台电机设定值给定
	PitchMotor.Location.SetLocation	=	GimbalData.PitchSetLocation;
	YawMotor.Location.SetLocation	=	GimbalData.YawSetLocation;
	
	if(GimbalData.FlagPitchUseEncoder)
		PitchMotor.Location.Location	=	EncoderDataSave.Pitch;
	else
		PitchMotor.Location.Location	=	GyroDataSave.Pitch;
	
	
	if(GimbalData.FlagYawUseEncoder)
		YawMotor.Location.Location	=	EncoderDataSave.Yaw;
	else
		YawMotor.Location.Location	=	GyroDataSave.Yaw;
	
	if(FlagPitchUseEncoderTemp!=GimbalData.FlagPitchUseEncoder)
	{
		PitchMotor.Location.SetLocation	=	PitchMotor.Location.Location;
		PitchSetLocationValueChange(PitchMotor.Location.SetLocation);
	}
	
	if(FlagYawUseEncoderTemp!=GimbalData.FlagYawUseEncoder)
	{
		YawMotor.Location.SetLocation	=	YawMotor.Location.Location;
		YawSetLocationValueChange(YawMotor.Location.SetLocation);
	}
	
	FlagPitchUseEncoderTemp=GimbalData.FlagPitchUseEncoder;
	FlagYawUseEncoderTemp = GimbalData.FlagYawUseEncoder;
	
}

extern FeedMotorStruct FeedMotor;
void GimbalControlCalculateAndSend(void)
{
	u8 Can2GimbalSendMessege[8];
	PitchMotor.PIDLocation.Ref	=	PitchMotor.Location.SetLocation;
	PitchMotor.PIDLocation.Fdb	=	PitchMotor.Location.Location;
	
	PitchMotor.PIDLocation.calc(&PitchMotor.PIDLocation);
	
	PitchMotor.Speed.SetSpeed	=	PitchMotor.PIDLocation.Out;
	
	PitchMotor.PIDSpeed.Ref	=	PitchMotor.Speed.SetSpeed;
	PitchMotor.PIDSpeed.Fdb	=	PitchMotor.Speed.Speed;
	
	PitchMotor.PIDSpeed.calc(&PitchMotor.PIDSpeed);
	
	YawMotor.PIDLocation.Ref	=	YawMotor.Location.SetLocation;
	YawMotor.PIDLocation.Fdb	=	YawMotor.Location.Location;
	
	YawMotor.PIDLocation.calc(&YawMotor.PIDLocation);
	
	YawMotor.Speed.SetSpeed	=	YawMotor.PIDLocation.Out;
	
	YawMotor.PIDSpeed.Ref	=	YawMotor.Speed.SetSpeed;
	YawMotor.PIDSpeed.Fdb	=	YawMotor.Speed.Speed;
	
	YawMotor.PIDSpeed.calc(&YawMotor.PIDSpeed);
	
	Can2GimbalSendMessege[0]	=	((s16)(-YawMotor.PIDSpeed.Out*4500))>>8;
	Can2GimbalSendMessege[1]	=	((s16)(-YawMotor.PIDSpeed.Out*4500))&0x00ff;
	
	Can2GimbalSendMessege[2]	=	((s16)(-PitchMotor.PIDSpeed.Out*4500))>>8;
	Can2GimbalSendMessege[3]	=	((s16)(-PitchMotor.PIDSpeed.Out*4500))&0x00ff;	
	
	Can2GimbalSendMessege[4]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))>>8;
	Can2GimbalSendMessege[5]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))&0x00ff;
	
#if DEBUG_USE_GIMBALMOTOR_CANSEND
	CAN2_Send_Msg(Can2GimbalSendMessege,8);
#endif
}

float GetGimbalError()
{
	return fabs(YawMotor.PIDLocation.Err)	+	fabs(PitchMotor.PIDLocation.Err);
}

void GimbalPIDClear()
{
	PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
	YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
}

void GimbalDataInit()
{
	PitchMotor.Location.SetLocation = PITCH_INIT_VALUE;
	YawMotor.Location.SetLocation = YAW_INIT_VALUE;
}
void PitchDataInit()
{
	PitchMotor.Location.SetLocation = PITCH_INIT_VALUE;
}
