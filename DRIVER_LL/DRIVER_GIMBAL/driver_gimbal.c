/****************************************************
*			Title:		��̨����
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
#define PITCH_SPEED_KP (0.5)							//20
#define PITCH_SPEED_KI (0)
#define PITCH_SPEED_KD (0)

#define YAW (0)

#define YAW_LOCATION_KP (0.1f)					//1.2
#define YAW_LOCATION_KI (0)
#define YAW_LOCATION_KD (0)
#define YAW_LOCATION_KC (0)
#define YAW_SPEED_KP (0.5)						//25
#define YAW_SPEED_KI (0)
#define YAW_SPEED_KD (0)


#define ROLL (0)

#define ROLL_LOCATION_KP (0.1f)					//1.2
#define ROLL_LOCATION_KI (0)
#define ROLL_LOCATION_KD (0)
#define ROLL_LOCATION_KC (0)
#define ROLL_SPEED_KP (1)						//25
#define ROLL_SPEED_KI (0)
#define ROLL_SPEED_KD (0)

GimbalMotorStruct	YawMotor,PitchMotor,RollMotor;
extern  RemoteDataUnion RemoteData;

void GimbalInit(void)
{
	YawMotor.Location.SetLocation=YAW_INIT_VALUE;
	PitchMotor.Location.SetLocation=PITCH_INIT_VALUE;	
	//Yaw��λ��PID��ʼ��
	YawMotor.PIDLocation.OutMax=1;
	YawMotor.PIDLocation.OutMin=-1;
	YawMotor.PIDLocation.calc=&PidCalc;
	YawMotor.PIDLocation.clear=&PidClear;
	YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
	
	YawMotor.PIDSpeed.OutMax=0.1;
	YawMotor.PIDSpeed.OutMin=-0.1;
	YawMotor.PIDSpeed.calc=&PidCalc;
	YawMotor.PIDSpeed.clear=&PidClear;
	YawMotor.PIDSpeed.clear(&YawMotor.PIDSpeed);
	
	PitchMotor.PIDLocation.OutMax=1;
	PitchMotor.PIDLocation.OutMin=-1;
	PitchMotor.PIDLocation.calc=&PidCalc;
	PitchMotor.PIDLocation.clear=&PidClear;
	PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
	
	PitchMotor.PIDSpeed.OutMax=0.1;
	PitchMotor.PIDSpeed.OutMin=-0.1;
	PitchMotor.PIDSpeed.calc=&PidCalc;
	PitchMotor.PIDSpeed.clear=&PidClear;
	PitchMotor.PIDSpeed.clear(&PitchMotor.PIDSpeed);
	
	RollMotor.PIDLocation.OutMax=1;
	RollMotor.PIDLocation.OutMin=-1;
	RollMotor.PIDLocation.calc=&PidCalc;
	RollMotor.PIDLocation.clear=&PidClear;
	RollMotor.PIDLocation.clear(&RollMotor.PIDLocation);
	
	RollMotor.PIDSpeed.OutMax=0.1;
	RollMotor.PIDSpeed.OutMin=-0.1;
	RollMotor.PIDSpeed.calc=&PidCalc;
	RollMotor.PIDSpeed.clear=&PidClear;
	RollMotor.PIDSpeed.clear(&RollMotor.PIDSpeed);


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
	
	RollMotor.PIDLocation.Kp	= ROLL_LOCATION_KP;
	RollMotor.PIDLocation.Ki	= ROLL_LOCATION_KI;
	RollMotor.PIDLocation.Kd	= ROLL_LOCATION_KD;
	RollMotor.PIDLocation.Kc	= ROLL_LOCATION_KC;
	
	RollMotor.PIDSpeed.Kp	=	ROLL_SPEED_KP;
	RollMotor.PIDSpeed.Ki	=	ROLL_SPEED_KI;
	RollMotor.PIDSpeed.Kd	=	ROLL_SPEED_KD;

	
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
int RPitchSpeed,RPitchSpeedk;//����JScopeͼ����ʾ��K�Ǵ��˲�֮���

void GimbalSpeedDataUpdate()									//��̨�ٶȸ���
{
	short  yawspeed,rollspeed,rpitchspeed;
	float   yawspeedf,rollspeedf,rpitchspeedf;
	yawspeedf = (Gyroscope.speed_z/*-GYROZ_OFFSET*/) /60;//����ÿ��
//	RPitchMotor.Speed.Speed =(Gyroscope.speed_y/*-GYROY_OFFSET*/) *6.28;
//	RPitchMotor.Speed.Speed	=	RPitchMotor.Speed.Speed*K	/	320+RPitchMotor.Speed.SpeedLast*(1-K);
//	RPitchMotor.Speed.SpeedLast=RPitchMotor.Speed.Speed;
	rpitchspeedf=(Gyroscope.speed_y/*-GYROY_OFFSET*/) /60;//����ÿ��
	rollspeedf=(Gyroscope.speed_x)/60;
#	if 1	//��ֵ�˲�
	PitchMotor.Speed.Speed=rpitchspeedf*K+PitchMotor.Speed.SpeedLast*(1-K);
	PitchMotor.Speed.SpeedLast=PitchMotor.Speed.Speed;
	YawMotor.Speed.Speed	=	yawspeedf*K	+YawMotor.Speed.SpeedLast*(1-K);
	YawMotor.Speed.SpeedLast=YawMotor.Speed.Speed;
	RollMotor.Speed.Speed=rollspeedf*K+RollMotor.Speed.SpeedLast*(1-K);
	RollMotor.Speed.SpeedLast=RollMotor.Speed.Speed;
	#endif
}
extern u8 GimbalInitFlag;
int YawGyroCount=0,PitchGyroCount=0,RollGyroCount=0;
GyroDataStruct YawPitchGyroDataUpdate(float YawData,float PitchData,float RollData)				//�����Ǹ���
{
	GyroDataStruct GyroData;
	float YawGyroDataTemp,PitchGyroDataTemp,RollDataTemp;
	static float YawGyroDataLast=0,PitchGyroDataLast=0,RollGyroDataLast=0;
	PitchGyroDataTemp	=	-PitchData;
	PitchGyroDataTemp/=360;

	YawGyroDataTemp	=	YawData;
	YawGyroDataTemp	=	YawGyroDataTemp	/	360.0f;
	
	RollDataTemp	=	RollData;
	RollDataTemp	=	RollDataTemp	/	360.0f;

	if (GimbalInitFlag)
	{
		YawGyroCount=0;
		if (PitchGyroDataTemp<0)
		PitchGyroCount=1;
		else 
		PitchGyroCount=0;
		RollGyroCount=0;
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
	GyroData.Pitch	=	PitchGyroDataTemp*K+PitchGyroDataLast*(1-K) + PitchGyroCount;//PITCH�޹����
 	PitchGyroDataLast= PitchGyroDataTemp;
	
	if			(RollDataTemp	-	RollGyroDataLast	>	0.8f)
	{RollGyroCount--;RollGyroDataLast++;}
	else if	(RollDataTemp	-	RollGyroDataLast	<	-0.8f)
	{RollGyroCount++;RollGyroDataLast--;}
	GyroData.Roll	=	RollDataTemp*K+RollGyroDataLast*(1-K) + RollGyroCount;
	RollGyroDataLast 	= RollDataTemp ;
	return GyroData;
}
  u16 a;
EncoderDataStruct YawPitchEncoderDataUpdate()										//���������ݸ���
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

void GyroAndEncoderDataGet(void)					//������ֵ��ʱ���£�1ms
{
//	float roll;
//	if(get_dmp_data(&MPU9250Yaw,&MPU9250Pitch,&roll)==0)
//	{
//			GyroDataSave = YawPitchGyroDataUpdate(MPU9250Yaw,MPU9250Pitch);
//	}
	HI229_Get_Gyroscope_Location(&Gyroscope.yaw,&Gyroscope.pitch,&Gyroscope.roll);
	HI229_Get_Gyroscope_Speed(&Gyroscope.speed_x,&Gyroscope.speed_y,&Gyroscope.speed_z);

	//EncoderDataSave	=	YawPitchEncoderDataUpdate();
	GyroDataSave = YawPitchGyroDataUpdate( Gyroscope.yaw,Gyroscope.pitch,Gyroscope.roll);

}
void YawSetLocationValueChange(float Yaw);
void PitchSetLocationValueChange(float Pitch);
void GimbalDataInput(GimbalSetLocationStruct GimbalData)
{
	static u8 FlagPitchUseEncoderTemp=1,FlagYawUseEncoderTemp=1;
	GimbalSetLocationData.YawSetLocation	=	GimbalData.YawSetLocation;
	GimbalSetLocationData.PitchSetLocation=	GimbalData.PitchSetLocation;
//��̨����趨ֵ����
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
	RollMotor.Location.Location=GyroDataSave.Roll;
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
float PitchError=0;
int RollError=0;
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
	YAWError=YawMotor.PIDSpeed.Out*10;
	#if 1//ʹ��pitch���
	PitchMotor.PIDLocation.Ref	=	PitchMotor.Location.SetLocation;
	PitchMotor.PIDLocation.Fdb	=	PitchMotor.Location.Location;
	PitchMotor.PIDLocation.calc(&PitchMotor.PIDLocation);
	PitchMotor.Speed.SetSpeed	=	PitchMotor.PIDLocation.Out;
	
	PitchMotor.PIDSpeed.Ref	=	PitchMotor.Speed.SetSpeed;
	PitchMotor.PIDSpeed.Fdb	=	PitchMotor.Speed.Speed;
	PitchMotor.PIDSpeed.calc(&PitchMotor.PIDSpeed);
	PitchError=PitchMotor.PIDSpeed.Out*10;
	
	RollMotor.PIDLocation.Ref=RollMotor.Location.SetLocation;
	RollMotor.PIDLocation.Fdb=RollMotor.Location.Location;
	RollMotor.PIDLocation.calc(&RollMotor.PIDLocation);
	RollMotor.Speed.SetSpeed	=	RollMotor.PIDLocation.Out;
	
	RollMotor.PIDSpeed.Ref	=	RollMotor.Speed.SetSpeed;
	RollMotor.PIDSpeed.Fdb	=	RollMotor.Speed.Speed;
	RollMotor.PIDSpeed.calc(&RollMotor.PIDSpeed);
	RollError=RollMotor.PIDSpeed.Out*2000;
	
	/************roll�ķ��ͷ�������**************/
#if 1 //���ùؿر���
	if (!RemoteLostCount)
	{
		LL_TIM_OC_SetCompareCH1(TIM12,630);
		LL_TIM_OC_SetCompareCH2(TIM12,630);
	}	
	else 
	{		
		LL_TIM_OC_SetCompareCH1(TIM12,630+RollError);
		LL_TIM_OC_SetCompareCH2(TIM12,630-RollError);
	}
	#else  //�رչؿر���
		LL_TIM_OC_SetCompareCH1(TIM12,630+RollError);
		LL_TIM_OC_SetCompareCH2(TIM12,630-RollError);
#endif
	
	#endif 
	
	
	#if 0 //û��CAN��������ע�� DEBUG_USE_GIMBALMOTOR_CANSEND
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