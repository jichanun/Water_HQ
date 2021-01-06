/****************************************************
*			Title:		云台控制
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#include "driver_gimbal.h"
#include "driver_icm20689.h"
#include "bsp_can.h"
#include "math.h"
#include "driver_feedmotor.h"
#include "driver_remote.h"
#include "driver_hi229um.h"
#include "task_remote.h"

#define ENCODER_LINE (8191)
#define GYROY_OFFSET (0)
#define GYROZ_OFFSET (0)

#define PITCH (1)

#define PITCH_LOCATION_KP (0.1)				//2
#define PITCH_LOCATION_KI (0)
#define PITCH_LOCATION_KD (0)
#define PITCH_SPEED_KP (10)							//20
#define PITCH_SPEED_KI (0)
#define PITCH_SPEED_KD (0)

#define YAW (0)

#define YAW_LOCATION_KP (1.3f)					//1.2
#define YAW_LOCATION_KI (0)
#define YAW_LOCATION_KD (0)
#define YAW_LOCATION_KC (0)
#define YAW_SPEED_KP (5)						//25
#define YAW_SPEED_KI (0)
#define YAW_SPEED_KD (0)

GimbalMotorStruct	YawMotor,PitchMotor;
extern  RemoteDataUnion RemoteData;

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
	
	LL_TIM_CC_EnableChannel(TIM12,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM12,LL_TIM_CHANNEL_CH2);	
	LL_TIM_EnableCounter(TIM12);
	LL_TIM_EnableAllOutputs(TIM12);

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
float K=0.1;
int RPitchSpeed,RPitchSpeedk;//用于JScope图像显示，K是带滤波之后的

void GimbalSpeedDataUpdate()									//云台速度更新
{
	short  yawspeed,lpitchspeed,rpitchspeed;
	float   yawspeedf,lpitchspeedf,rpitchspeedf;
	yawspeedf = (Gyroscope.speed_z/*-GYROZ_OFFSET*/) /60;//弧度每分
//	RPitchMotor.Speed.Speed =(Gyroscope.speed_y/*-GYROY_OFFSET*/) *6.28;
//	RPitchMotor.Speed.Speed	=	RPitchMotor.Speed.Speed*K	/	320+RPitchMotor.Speed.SpeedLast*(1-K);
//	RPitchMotor.Speed.SpeedLast=RPitchMotor.Speed.Speed;
	rpitchspeedf=(Gyroscope.speed_y/*-GYROY_OFFSET*/) /60;//弧度每分
	#if 1	//均值滤波
	PitchMotor.Speed.Speed=rpitchspeedf*K+PitchMotor.Speed.SpeedLast*(1-K);
	PitchMotor.Speed.SpeedLast=PitchMotor.Speed.Speed;
	YawMotor.Speed.Speed	=	yawspeedf*K	+YawMotor.Speed.SpeedLast*(1-K);
	YawMotor.Speed.SpeedLast=YawMotor.Speed.Speed;

	#endif
}
extern u8 GimbalInitFlag;
int YawGyroCount=0,PitchGyroCount=0;
GyroDataStruct YawPitchGyroDataUpdate(float YawData,float PitchData)				//陀螺仪更新
{
	GyroDataStruct GyroData;
	float YawGyroDataTemp,PitchGyroDataTemp;
	static float YawGyroDataLast=0,PitchGyroDataLast=0;
	PitchGyroDataTemp	=	-PitchData;
	PitchGyroDataTemp/=360;

	YawGyroDataTemp	=	YawData;
	YawGyroDataTemp	=	YawGyroDataTemp	/	360.0f;

	if (GimbalInitFlag)
	{
		YawGyroCount=0;
		if (PitchGyroDataTemp<0)
		PitchGyroCount=1;
		else 
		PitchGyroCount=0;
	}

	if			(YawGyroDataTemp	-	YawGyroDataLast	>	0.8f)
	{YawGyroCount--;YawGyroDataLast++;}
	else if	(YawGyroDataTemp	-	YawGyroDataLast	<	-0.8f)
	{YawGyroCount++;YawGyroDataLast--;}
	GyroData.Yaw	=	YawGyroDataTemp*K+YawGyroDataLast*(1-K) + YawGyroCount;
	YawGyroDataLast 	= YawGyroDataTemp ;
	
	if			(PitchGyroDataTemp	-	PitchGyroDataLast	>	0.8f)
	{PitchGyroCount--;PitchGyroDataLast++;}
	else if	(PitchGyroDataTemp	-	PitchGyroDataLast	<	-0.8f)
	{PitchGyroCount++;PitchGyroDataLast--;}
	GyroData.Pitch	=	PitchGyroDataTemp*K+PitchGyroDataLast*(1-K) + PitchGyroCount;//PITCH无过零点
 	PitchGyroDataLast= PitchGyroDataTemp;
	return GyroData;
}
  u16 a;
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
//	float roll;
//	if(get_dmp_data(&MPU9250Yaw,&MPU9250Pitch,&roll)==0)
//	{
//			GyroDataSave = YawPitchGyroDataUpdate(MPU9250Yaw,MPU9250Pitch);
//	}
	HI229_Get_Gyroscope_Location(&Gyroscope.yaw,&Gyroscope.pitch,&Gyroscope.roll);
	HI229_Get_Gyroscope_Speed(&Gyroscope.speed_x,&Gyroscope.speed_y,&Gyroscope.speed_z);

	//EncoderDataSave	=	YawPitchEncoderDataUpdate();
	GyroDataSave = YawPitchGyroDataUpdate( Gyroscope.yaw,Gyroscope.pitch);

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
float YAWSpinK=1;
//extern RemoteDataPortStruct	RemoteDataPort;
float YAWError=0;
int PitchError=1000;
extern  int RemoteLostCount;
void GimbalControlCalculateAndSend(void)
{
	u8 Can2GimbalSendMessege[8];

	YawMotor.PIDLocation.Ref	=	YawMotor.Location.SetLocation;
	YawMotor.PIDLocation.Fdb	=	YawMotor.Location.Location;
	YawMotor.PIDLocation.calc(&YawMotor.PIDLocation);
	YawMotor.Speed.SetSpeed	=	YawMotor.PIDLocation.Out;
	
	YawMotor.PIDSpeed.Ref	=	YawMotor.Speed.SetSpeed;
	YawMotor.PIDSpeed.Fdb	=	YawMotor.Speed.Speed;	
	YawMotor.PIDSpeed.calc(&YawMotor.PIDSpeed);
	YAWError=YawMotor.PIDSpeed.Out;
//	Can2GimbalSendMessege[0]	=	((s16)(YawMotor.PIDSpeed.Out*30000))>>8;
//	Can2GimbalSendMessege[1]	=	((s16)(YawMotor.PIDSpeed.Out*30000))&0x00ff;
	#if 1//使用pitch电机
	PitchMotor.PIDLocation.Ref	=	PitchMotor.Location.SetLocation;
	PitchMotor.PIDLocation.Fdb	=	PitchMotor.Location.Location;
	PitchMotor.PIDLocation.calc(&PitchMotor.PIDLocation);
	PitchMotor.Speed.SetSpeed	=	PitchMotor.PIDLocation.Out;
	
	PitchMotor.PIDSpeed.Ref	=	PitchMotor.Speed.SetSpeed;
	PitchMotor.PIDSpeed.Fdb	=	PitchMotor.Speed.Speed;
	PitchMotor.PIDSpeed.calc(&PitchMotor.PIDSpeed);
	PitchError=PitchMotor.PIDSpeed.Out*PitchError+1000;
#if 1 //启用关控保护
	if (!RemoteLostCount)
	{
		LL_TIM_OC_SetCompareCH1(TIM12,0);
		LL_TIM_OC_SetCompareCH2(TIM12,0);
	}	
	else 
	{		
		LL_TIM_OC_SetCompareCH1(TIM12,PitchError);
		LL_TIM_OC_SetCompareCH2(TIM12,PitchError);
	}
	#else  //关闭关控保护
		LL_TIM_OC_SetCompareCH1(TIM12,PitchError);
		LL_TIM_OC_SetCompareCH2(TIM12,PitchError);
#endif
	
	#endif 
	
	
	#if 0 //没有CAN发送所以注释 DEBUG_USE_GIMBALMOTOR_CANSEND
	if(RemoteData.RemoteDataProcessed.RCValue.s2==2)
	{
	  Can2GimbalSendMessege[0]	=	0;
	  Can2GimbalSendMessege[1] = 0;
		CAN2_Send_Msg(Can2GimbalSendMessege,8);
	}
		else
		{
	CAN2_Send_Msg(Can2GimbalSendMessege,8);
		}
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
