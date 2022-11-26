/****************************************************
*			Title:		��̨����
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#ifndef __DRIVER_GIMBAL_H__
#define __DRIVER_GIMBAL_H__
#include "sys.h"
#include "pid.h"
#include "BSPconfig.h"
#define PITCH_INIT_VALUE PITCH_INIT_VALUE_SET		
#define PITCH_TEMPSET (PITCH_INIT_VALUE>0.5f ? 1.0f:0.0f)
#define YAW_INIT_VALUE YAW_INIT_VALUE_SET 		
#define YAW_TEMPSET	(YAW_INIT_VALUE>0.5f ? 1.0f:0.0f)

#define PITCH_MAX_VALUE (PITCH_INIT_VALUE+0.04f)
#define PITCH_MIN_VALUE (PITCH_INIT_VALUE-0.06f)

#define YAW_MAX_VALUE (YAW_INIT_VALUE+0.16f)
#define YAW_MIN_VALUE (YAW_INIT_VALUE-0.16f)

#define YAW_TARGET_MODE0  120  //ǰ��ս
#define YAW_TARGET_MODE1  125  //����


void GimbalInit(void);
void GimbalReturnToInitLocation(u8 IfPitch,u8 IfYaw);
void GimbalPIDLocationClear(u8 IfPitch,u8 IfYaw);

typedef struct					//����ٶ����ֵ
{
	float	SetSpeed;
	float Speed;
	float SpeedLast;
}SpeedStruct;						//�ٶȽṹ�嶨��

typedef struct					//���λ�����ֵ
{
	float	SetLocation;
	float Location;
}LocationStruct;				//λ�ýṹ�嶨��

typedef struct
{
	LocationStruct	Location;
	PID	PIDLocation;
	SpeedStruct	Speed;
	PID	PIDSpeed;
	u8 CANReceiveMessege[8];
	int RollError;
	int RollSink;
	
}GimbalMotorStruct;

typedef struct
{
	float Yaw;
	float Pitch;
	float Roll;
}GyroDataStruct;

typedef struct
{
	float Yaw;
	float Pitch;
}EncoderDataStruct;

typedef struct
{
	float PitchSetLocation;
	float YawSetLocation;
	float RollSetLocation;
	u8 FlagPitchUseEncoder;
	u8 FlagYawUseEncoder;
}GimbalSetLocationStruct;
typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float speed_x;
	float speed_y;
	float speed_z;
	float ax;
	float ay;
	float az;
	float vx;
	float vy;
	float vz;
}GyroscopeStruct;

float GetYawEncoderValue(void);
float GetYawGyroValue(void);
float GetYawLocation(void);
float GetPitchLocation(void);
float GetRollLocation(void);
float ReturnGX(void);
float ReturnGY(void);
float ReturnGZ(void);

void GimbalPIDClear(void);
void GimbalDataInit(void);
void PitchDataInit(void);
typedef struct 
{
	u8 circle;
	u8 square;
	
}StatusCountStruct;
typedef struct 
{
	float rho;
	float rho_last;
	float angle;
	float angle_last;
	float change_rho;
	float change_angle;
	float change_angle_last;
	float error_x;
	float error_y;
	u8 status;
	u8 statusfinal;
	float yaw_offset;
	float rho_offset;
	StatusCountStruct statuscount;
}VisionDataStruct;

extern GyroscopeStruct Gyroscope;

void GimbalControlTask(void);
void GyroAndEncoderDataGet(void);
EncoderDataStruct YawPitchEncoderDataUpdate(void);
GyroDataStruct YawPitchGyroDataUpdate(float YawData,float PitchData,float RollData)	;			
void GimbalDataInput(GimbalSetLocationStruct GimbalData);
void GimbalControlCalculateAndSend(void);
void GyroAndEncoderDataGet(void);					//������ֵ��ʱ���£�1ms
void GimbalSpeedDataUpdate(void);									//��̨�ٶȸ���
void VisionInit(void);
void uart3WriteBuf(uint8_t*SendBuf,int len);//����3�����ַ���
#endif
