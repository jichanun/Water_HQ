#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"
#include "driver_remote.h"
#include "math.h"
//遥控器灵敏度
#define YAW_REMOTE_SENSITIVE (0.0005f)
#define PITCH_REMOTE_SENSITIVE (0.0005f)
#define ONE1 0x3F800000
#define TWO1 0x800000

GimbalSetLocationStruct	GimbalSetLocationDataTemp;
extern  RemoteDataUnion RemoteData;
extern  FeedMotorStruct FeedMotor;
extern  SwitchStruct Switch;
extern  int  Target_mode;
extern u8 VisionReceiveFlag;
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
	if (VisionReceiveFlag)
		VisionControl();
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
extern u8 UART2BUFF[15];
u32 VisionReceiveData[2]={0};
float VisionData[2]={0};
void UART2Unpack(u8 *buff,u32 *num)
{
	int a ;
	for (int i =0;i<2;i++)
	{
		u32 a  = ((buff[i*4+2]<<24)|(buff[i*4+3]<<16)|(buff[i*4+4]<<8)|(buff[i*4+5]));
		int  n = (a-ONE1)/TWO1;
		u32 x = 1<<n;
		num[i]=x+(a-ONE1-TWO1*n)*x/TWO1;
	}
}
void VisionControl(void)
{
	VisionReceiveFlag=0;
	if (UART2BUFF[0]==0XFF&&UART2BUFF[1]==0XFF)
		UART2Unpack(UART2BUFF,VisionReceiveData);
	VisionData[0]=(float)VisionReceiveData[0]-330;//偏离值
	VisionData[1]=(float)atan((VisionReceiveData[1]-320)/140)*57.3;//角度值
	
}