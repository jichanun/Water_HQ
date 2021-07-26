#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"
#include "driver_remote.h"
#include "math.h"
#include "driver_laser.h"
#include "driver_chassis.h"


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
VisionDataStruct VisionData;
extern u8 UART3BUFF[20];
u8 UART3BUFFLast[20];
u32 VisionReceiveData[4]={0};

void VisionReceiveDataClear(VisionDataStruct *Data)
{
	Data->angle_last=Data->angle;
	Data->rho_last=Data->rho;
	Data->angle=0;
	Data->rho=0;
	for(int i =0;i<20;i++)
	{
		UART3BUFFLast[i]=UART3BUFF[i];
		UART3BUFF[i]=0;
	}
}

void GimbalControlTask()
{
	//Step1:Get Gyro and encoder value
	GyroAndEncoderDataGet();
	//Step2:Get UWB value 
	if(GimbalInitFlag)
	{
		GimbalInitFlag--;
		GimbalSetLocationDataTemp.PitchSetLocation	=	PITCH_INIT_VALUE;
		GimbalSetLocationDataTemp.YawSetLocation	=	YAW_INIT_VALUE;
		GimbalSetLocationDataTemp.FlagPitchUseEncoder	=	0;
		GimbalSetLocationDataTemp.FlagYawUseEncoder	=	0;
		yaw_OFFSET =Gyroscope.yaw/360;
		GimbalSetLocationDataTemp.YawSetLocation	=	yaw_OFFSET;//使用陀螺仪则取消注释
		//此时向前运动
	}
	if (VisionReceiveFlag)
		VisionControl();
	//VisionReceiveDataClear(&VisionData);//接收重置

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
/**************
通信协议
偏移量			角度值 			x坐标 			y坐标				状态码
*///////////////
void UART3Unpack(u8 *buff,u32 *num)
{
	for (int i =0;i<4;i++)
	{
		u32 a  = ((buff[i*4+2]<<24)|(buff[i*4+3]<<16)|(buff[i*4+4]<<8)|(buff[i*4+5]));
		int  n = (a-ONE1)/TWO1;
		u32 x = 1<<n;
		num[i]=x+(a-ONE1-TWO1*n)*x/TWO1;
	}
}
void UART3Pack(u32 *num,u8 *txbuff)//a是那个十进制数x是返回的16进制数
{
	for (int j =0;j<4;j++)
	{
//		int i=1,n=0;
//		while(num[j]>=(i<<1)){
//			n++;
//			i=i<<1;
//		 }
//		 int x=TWO1*n+ONE1+TWO1/i*(num[j]-i);
		 txbuff[j*4]=num[j]>>24;
		 txbuff[j*4+1]=num[j]>>16;
		 txbuff[j*4+2]=num[j]>>8;
		 txbuff[j*4+3]=num[j]&0xff;
	 }
	 
}
u8 UART3TXBUFF[20];
u32 VisionTransmitData[4];
extern PositionDataStruct PositionStruct;


void VisionTransmit(void)
{
	//UART3TXBUFF[0]=UART3TXBUFF[1]=0XFF;
	VisionTransmitData[0]=10000;
	VisionTransmitData[1]=PositionStruct.yaw_error;//角度
	VisionTransmitData[2]=PositionStruct.actual_x*1000;//x（m*1000）
	VisionTransmitData[3]=PositionStruct.actual_y*1000;//y（m*1000）
	UART3Pack(VisionTransmitData,UART3TXBUFF);
	UART3TXBUFF[16]=PositionStruct.status;//标志位(为1 可用)
	uart3WriteBuf(UART3TXBUFF,17);
	
}
float FilterK=0.05;
int16_t TurnFlag=0;
int down_error=30;
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
		VisionData.rho=(float)VisionReceiveData[0]-330+VisionData.rho_offset;//偏离值
		VisionData.angle=(float)atan(a)*57.3+VisionData.yaw_offset;//角度值
		VisionData.angle=VisionData.angle*FilterK+VisionData.angle*(1-FilterK);
		///////下面改成位置坐标的XY轴
		VisionData.error_x=(int)(-VisionReceiveData[2]);//深度
		VisionData.error_y=VisionReceiveData[3];

	#if  0			////////////////使用滤波
		VisionData.status=UART3BUFF[18];
		switch (VisionData.status)
		{
			case 'C':
				VisionData.statuscount.circle++;
			break;
			case 'S':
				VisionData.statuscount.square++;
			break;
			case 'N':
			{
				VisionData.statuscount.circle=VisionData.statuscount.square=0;
			VisionData.statusfinal='N';
				break;
			}
			default:
				break;
		}
		if (VisionData.statuscount.circle>3)
		{
			VisionData.statusfinal='C';
			VisionData.statuscount.circle=VisionData.statuscount.square=0;
		}
		else if (VisionData.statuscount.square>3)
		{
			VisionData.statusfinal='S';
			VisionData.statuscount.circle=VisionData.statuscount.square=0;
		}
	#else         ///////////不使用滤波
		VisionData.statusfinal=UART3BUFF[18];
	#endif 
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
					//这些注释是用来弯道快速转弯
//					if (VisionData.angle>50&&TurnFlag<0&&fabs(VisionData.angle-VisionData.angle_last)<20){
//						YawSetLocationValueChange(-0.25);
//						TurnFlag=60;
//					}
//					else if (VisionData.angle<-50&&TurnFlag<0&&fabs(VisionData.angle-VisionData.angle_last)<20)
//					{
//						YawSetLocationValueChange(-0.25);
//						TurnFlag=60;
//					}
//					else 
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
		}
		
		
	}
//	else 				RollMotor.RollSink=0;

}
