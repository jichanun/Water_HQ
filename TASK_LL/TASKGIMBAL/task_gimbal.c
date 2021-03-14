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
float Rho_Maximum=500;
GimbalSetLocationStruct	GimbalSetLocationDataTemp;
extern PID VisionRhoIncreasement , VisionYawIncreasement ;
extern  RemoteDataUnion RemoteData;
extern  FeedMotorStruct FeedMotor;
extern  SwitchStruct Switch;
extern  int  Target_mode;
extern u8 VisionReceiveFlag;
extern  GimbalMotorStruct	YawMotor,PitchMotor,RollMotor;
extern float VisionRho;
extern u8 AutomaticAiming;
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
extern u8 UART3BUFF[15];
u8 UART3BUFFLast[15];
u32 VisionReceiveData[2]={0};
VisionDataStruct VisionData;
void UART3Unpack(u8 *buff,u32 *num)
{
	for (int i =0;i<2;i++)
	{
		u32 a  = ((buff[i*4+2]<<24)|(buff[i*4+3]<<16)|(buff[i*4+4]<<8)|(buff[i*4+5]));
		int  n = (a-ONE1)/TWO1;
		u32 x = 1<<n;
		num[i]=x+(a-ONE1-TWO1*n)*x/TWO1;
	}
}
void VisionReceiveDataClear(VisionDataStruct *Data)
{
	Data->angle_last=Data->angle;
	Data->rho_last=Data->rho;
	Data->angle=0;
	Data->rho=0;
	for(int i =0;i<15;i++)
	{
		UART3BUFFLast[i]=UART3BUFF[i];
		UART3BUFF[i]=0;
	}
}
float FilterK=0.05;
int16_t TurnFlag=0;
void VisionControl(void)
{
	VisionReceiveFlag=0;
	int b ;
	
	if (UART3BUFF[0]==0XFF&&UART3BUFF[1]==0XFF)
	{
		UART3Unpack(UART3BUFF,VisionReceiveData);
		b=VisionReceiveData[1]-320;
		float a =b;
		a/=130;
		VisionData.rho=(float)VisionReceiveData[0]-330;//偏离值
		VisionData.angle=(float)atan(a)*57.3;//角度值
		VisionData.angle=VisionData.angle*FilterK+VisionData.angle*(1-FilterK);
		
	#if 1 //视觉处的变结构PID
		VisionRhoIncreasement.Ref=VisionData.rho;
		VisionRhoIncreasement.calc(&VisionRhoIncreasement);
		VisionData.change_rho=VisionRhoIncreasement.Out;
		//计算有问题
		VisionYawIncreasement.Ref=VisionData.angle;
		VisionYawIncreasement.calc(&VisionYawIncreasement);
		VisionData.change_angle=VisionYawIncreasement.Out/500;
	#endif 
		//视觉处理****************************************************
			if(AutomaticAiming&&VisionData.rho!=-330)
		{
//			if((VisionData.change_angle<=25.0f*VisionYawIncreasement.Kp&&VisionData.change_angle>=3.0f*VisionYawIncreasement.Kp)||
//					(VisionData.change_angle>=-25.0f*VisionYawIncreasement.Kp&&VisionData.change_angle<=-3.0f*VisionYawIncreasement.Kp))
			{
				if(fabs(YawMotor.Location.SetLocation-YawMotor.Location.Location)<0.05)
				{
					if (VisionData.angle>50&&TurnFlag<0&&fabs(VisionData.angle-VisionData.angle_last)<20){
						YawSetLocationValueChange(-0.25);
						TurnFlag=60;
					}
					else if (VisionData.angle<-50&&TurnFlag<0&&fabs(VisionData.angle-VisionData.angle_last)<20)
					{
						YawSetLocationValueChange(-0.25);
						TurnFlag=60;
					}
					else 
					{
						YawSetLocationValueChange(-VisionData.change_angle);
						if (TurnFlag>-10)
						TurnFlag--;
					}
				}
			}
//		if((VisionData.change_rho<=12.5f*VisionYawIncreasement.Kp&&VisionData.change_rho>=0.5f*VisionYawIncreasement.Kp)||
//				(VisionData.change_rho>=-12.5f*VisionYawIncreasement.Kp&&VisionData.change_rho<=-0.5f*VisionYawIncreasement.Kp))
				VisionRho=VisionData.change_rho/Rho_Maximum;
			if (YawMotor.Location.SetLocation-yaw_OFFSET<-0.25)
				RollMotor.RollSink=-00;
			else 
				RollMotor.RollSink=0;
		}
		VisionReceiveDataClear(&VisionData);//接收重置
	}
//	else 				RollMotor.RollSink=0;

}
