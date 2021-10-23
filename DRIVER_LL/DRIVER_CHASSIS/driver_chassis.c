/****************************************************
*			Title:		Chassis
*			ChipType:	STM32F405RGT6
*			Version:	1.0.7
*			Date:			2017.09.14
*												LD.
*****************************************************/

#include "driver_chassis.h"
#include "bsp_can.h"
#include "driver_gimbal.h"
#include "math.h"
#include "uwb.h" 
#include "task_uwb.h" 

ChassisMotorStruct ChassisMotor[4];

#define AUTO_CALIBRATE 0//自动校准

//电机参数宏定义
#define	CHASSIS_MOTOR_KP	(0.8f)
#define	CHASSIS_MOTOR_KI	(0.0f)
#define	CHASSIS_MOTOR_KD	(0.0f)

#define CHASSIS_FOLLOW_KP	(8.0f)
#define CHASSIS_FOLLOW_KI	(0)
#define CHASSIS_FOLLOW_KD	(14.0f)
#define CHASSIS_NOT_FOLLOW_TOLERANCE (0.003f)

#define CHASSIS_FOLLOW_INIT_VALUE YAW_INIT_VALUE

#define SHAKECORRECTIONVALUE (1.2f)		//修正系数

#define PIE (3.1415926f)


#define POSITION_X_KP (-1)
#define POSITION_X_KI (0)
#define POSITION_X_KD (0)
//#define POSITION_X_AP (0)
//#define POSITION_X_BP (0)
//#define POSITION_X_CP (0)

#define POSITION_Y_KP (1)
#define POSITION_Y_KI (0)
#define POSITION_Y_KD (0)
//#define POSITION_Y_AP (0)
//#define POSITION_Y_BP (0)
//#define POSITION_Y_CP (0)
#define PX_MIN (0.5)
#define PY_MIN (0.5)
/*
底盘运动控制规律
电机摆放顺序;

M3   M4

M2   M1

电机：		M4	M1	M2	M3 
前进：		+		+		+		+
后退：		-		-		-		-
右平移：	-		+		-		+
左平移：	+		-		+		-
顺自旋：	-		-		+		+
逆自旋：	+		+		-		-
定义v,h,spin为垂直，水平，自旋运动控制归一量，前进、右移、顺时针自旋为正
电机：			M1	M2	M3	M4
y						+		+		+		+
x						+		-		+		-
spin				-		+		+		-
*/

PositionDataStruct PositionStruct;
PID PositionPID[2];
extern int RemoteLostCount;



struct
{
	float SetFollowValue;
	float FollowValue;
	PID PIDChassisFollow;
}ChassisFollow;
void ChassisInit(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		ChassisMotor[i].PIDSpeed.Kp=CHASSIS_MOTOR_KP;
		ChassisMotor[i].PIDSpeed.Ki=CHASSIS_MOTOR_KI;
		ChassisMotor[i].PIDSpeed.Kd=CHASSIS_MOTOR_KD;
		ChassisMotor[i].PIDSpeed.calc=&PidCalc;
		ChassisMotor[i].PIDSpeed.clear=&PidClear;
		ChassisMotor[i].PIDSpeed.OutMax=1;
		ChassisMotor[i].PIDSpeed.OutMin=-1;
		ChassisMotor[i].PIDSpeed.clear(&ChassisMotor[i].PIDSpeed);
	}
	ChassisFollow.PIDChassisFollow.Kp	=	CHASSIS_FOLLOW_KP;
	ChassisFollow.PIDChassisFollow.Ki	=	CHASSIS_FOLLOW_KI;
	ChassisFollow.PIDChassisFollow.Kd	=	CHASSIS_FOLLOW_KD;
	
	ChassisFollow.PIDChassisFollow.OutMax	=	1;
	ChassisFollow.PIDChassisFollow.OutMin	=	-1;
	ChassisFollow.PIDChassisFollow.calc=&PidCalc;
	ChassisFollow.PIDChassisFollow.clear=&PidClear;
	ChassisFollow.PIDChassisFollow.clear(&ChassisFollow.PIDChassisFollow);
	
	ChassisFollow.SetFollowValue	=	CHASSIS_FOLLOW_INIT_VALUE;
		PositionPID[0].Kp	= POSITION_X_KP;
	PositionPID[0].Ki	= POSITION_X_KI;
	PositionPID[0].Kd	= POSITION_X_KD;
	PositionPID[0].OutMax=0.5;
	PositionPID[0].OutMin=-0.5;
	PositionPID[0].calc=&PidCalc;
	PositionPID[0].clear=&PidClear;
	PositionPID[0].clear(&PositionPID[0]);
	
	PositionPID[1].Kp	= POSITION_Y_KP;
	PositionPID[1].Ki	= POSITION_Y_KI;
	PositionPID[1].Kd	= POSITION_Y_KD;
	PositionPID[1].OutMax=0.5;
	PositionPID[1].OutMin=-0.5;
	PositionPID[1].calc=&PidCalc;
	PositionPID[1].clear=&PidClear;
	PositionPID[1].clear(&PositionPID[1]);

}
void ChassisFollowInit()
{
	ChassisFollow.SetFollowValue = CHASSIS_FOLLOW_INIT_VALUE ;
}

void SetChassisFollowRefTargetValue()
{
	ChassisFollow.SetFollowValue = CHASSIS_FOLLOW_INIT_VALUE + 0.132f;
}

void SetChassisFollowRef(float SetLocation)
{
	ChassisFollow.SetFollowValue	=	SetLocation;
}

void ChassisFollowCalculate(ChassisSpeedMessegePort *ChassisSpeed)
{
	ChassisFollow.FollowValue	=	GetYawEncoderValue();
	
	ChassisFollow.PIDChassisFollow.Ref	=	ChassisFollow.SetFollowValue;
	ChassisFollow.PIDChassisFollow.Fdb	=	ChassisFollow.FollowValue;
	
	ChassisFollow.PIDChassisFollow.calc(&ChassisFollow.PIDChassisFollow);
	
	if(fabs(ChassisFollow.PIDChassisFollow.Err)>CHASSIS_NOT_FOLLOW_TOLERANCE)
		ChassisSpeed->Spin =	-ChassisFollow.PIDChassisFollow.Out;
	else
		ChassisSpeed->Spin = 0 ;
}

/*******************shake kinematics anlysis****************************
		正方向旋转a角----则有
		[	speedy'	]	=	[	cos a		-sin a	]	[	speedy	]
		[	speedx'	]		[	sin a		cos a		]	[	speedx	]
		正方向角度与err相反 修正坐标旋转矩阵
		[	speedy'	]	=	[	cos a		sin a	]	[	speedy	]
		[	speedx'	]		[	-sin a	cos a	]	[	speedx	]
***********************************************************************/
void	ChassisShakeCalculate(ChassisSpeedMessegePort *ChassisSpeed)
{
	float	Angle,CosAngle,SinAngle,SpeedXTemp,SpeedYTemp;
	static float ShakeValue;
	Angle	=	ChassisFollow.PIDChassisFollow.Err*PIE*SHAKECORRECTIONVALUE;
	
	CosAngle=cos(Angle);
	SinAngle=sin(Angle);
	
	SpeedXTemp	=	ChassisSpeed	->	SetSpeedX;
	SpeedYTemp	=	ChassisSpeed	->	SetSpeedY;
	
	ChassisSpeed	->	SetSpeedY	=	CosAngle	*	SpeedYTemp	+	-SinAngle	*		SpeedXTemp;
	ChassisSpeed	->	SetSpeedX	=	SinAngle	*	SpeedYTemp	+	CosAngle	*		SpeedXTemp;
	
	ShakeValue +=	PIE/160;
	
	ChassisSpeed	->	Spin	+=	sin(ShakeValue)/1.1;
}

void	ChassisChangeFollow(ChassisSpeedMessegePort *ChassisSpeed)
{
	float	Angle,CosAngle,SinAngle,SpeedXTemp,SpeedYTemp;
	Angle	=	(CHASSIS_FOLLOW_INIT_VALUE	-	ChassisFollow.FollowValue)*PIE*SHAKECORRECTIONVALUE;
	CosAngle=cos(Angle);
	SinAngle=sin(Angle);
	
	SpeedXTemp	=	ChassisSpeed	->	SetSpeedX;
	SpeedYTemp	=	ChassisSpeed	->	SetSpeedY;
	
	ChassisSpeed	->	SetSpeedY	=	CosAngle	*	SpeedYTemp	+	-SinAngle	*		SpeedXTemp;
	ChassisSpeed	->	SetSpeedX	=	SinAngle	*	SpeedYTemp	+	CosAngle	*		SpeedXTemp;
}

void ChassisControl(ChassisSpeedMessegePort ChassisSpeed)
{
	u8 i;
	u8 CAN1SendMessegeBuffer[8];
	
	ChassisMotor[0].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[1].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		-	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[2].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		- 1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[3].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	
	for(i=0;i<4;i++)
	{
		ChassisMotor[i].Speed.Speed=((int16_t)((ChassisMotor[i].SpeedReceiveMessege[2]<<8)|ChassisMotor[i].SpeedReceiveMessege[3]));
		ChassisMotor[i].Speed.Speed = ChassisMotor[i].Speed.Speed/9600;
		
		if(i==1||i==2)
			ChassisMotor[i].Speed.SetSpeed=-ChassisMotor[i].Speed.SetSpeed;
		
		ChassisMotor[i].PIDSpeed.Ref=ChassisMotor[i].Speed.SetSpeed;
		ChassisMotor[i].PIDSpeed.Fdb=ChassisMotor[i].Speed.Speed;
		
		ChassisMotor[i].PIDSpeed.calc(&ChassisMotor[i].PIDSpeed);
		
		CAN1SendMessegeBuffer[i*2]=((int16_t)(ChassisMotor[i].PIDSpeed.Out*32767))>>8;
		CAN1SendMessegeBuffer[i*2+1]=((int16_t)(ChassisMotor[i].PIDSpeed.Out*32767))&0x00ff;
		
	}
#if DEBUG_USE_CHASSISMOTOR_CANSEND
	CAN1_Send_Msg(CAN1SendMessegeBuffer,8);
#endif
}
extern int RemoteLostCount;
float ChassisSpeedK=210;	
u16 speed0=0,speed1=0,speed2=0,speed3=0;
float ReverseError=1.1;
float ChassisSpeedMax=0.5;//※※※※※※※※※※速度最大值0.5
void ChassisControl_PWM(ChassisSpeedMessegePort ChassisSpeed)
{
	ChassisMotor[0].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[1].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		-	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[2].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		- 1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[3].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	for(int i=0;i<4;i++)
	{
		if (ChassisMotor[i].Speed.SetSpeed>ChassisSpeedMax)
			ChassisMotor[i].Speed.SetSpeed=ChassisSpeedMax;
		else if (ChassisMotor[i].Speed.SetSpeed<-ChassisSpeedMax)
			ChassisMotor[i].Speed.SetSpeed=-ChassisSpeedMax;
		if (i==1||i==2)
		{
			if (ChassisMotor[i].Speed.SetSpeed<0)
				ChassisMotor[i].Speed.SetSpeed*=ReverseError;//反转倍率
		}
		else 
			if (ChassisMotor[i].Speed.SetSpeed>0)
				ChassisMotor[i].Speed.SetSpeed*=ReverseError;//反转倍率
	}
	
#if 1 //启用关控保护
	if (!RemoteLostCount)
	{
		LL_TIM_OC_SetCompareCH1(TIM2,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH2(TIM2,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH3(TIM8,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH4(TIM8,MIDDLE_PWM);
	}	
	else 
	{		
		#if  (SELF_ID==3||SELF_ID==2)
			speed0=(s16)(-(ChassisMotor[0].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
			speed1=(s16)(-(ChassisMotor[1].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
			speed2=(s16)(-(ChassisMotor[2].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
			speed3=(s16)(-(ChassisMotor[3].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		#elif  SELF_ID==0;
			speed0=(s16)(-(ChassisMotor[0].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
			speed1=(s16)(-(ChassisMotor[1].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
			speed2=(s16)(-(ChassisMotor[2].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
			speed3=(s16)(-(ChassisMotor[3].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		#endif
		LL_TIM_OC_SetCompareCH1(TIM2,speed3);
		LL_TIM_OC_SetCompareCH2(TIM2,speed2);
		LL_TIM_OC_SetCompareCH3(TIM8,speed0);
		LL_TIM_OC_SetCompareCH4(TIM8,speed1);
	}
	#else  //关闭关控保护
		speed0=(u16)((ChassisMotor[0].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed1=(u16)((ChassisMotor[1].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed2=(u16)((ChassisMotor[2].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed3=(u16)((ChassisMotor[3].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH1(TIM2,speed2);
		LL_TIM_OC_SetCompareCH2(TIM2,speed3);
		LL_TIM_OC_SetCompareCH3(TIM8,speed1);
		LL_TIM_OC_SetCompareCH4(TIM8,speed0);
#endif
	
}

float yaw_Base = 0;
extern VisionDataStruct VisionData;
extern API_Struct Position_LL;

extern  float yaw_OFFSET;

u16 PositionCount=0;

void Position_Init(ChassisSpeedMessegePort *ChassisSpeed)
{
	static float PX,PY,theta;
	if(Position_LL.Validation&&!Position_LL.validp&&RemoteLostCount)//遥控器不丢数据，角度不准，定位准
	{
		//PositionStruct.status=0;//发送未完成信号
	#if !AUTO_CALIBRATE //手动校准
		Position_LL.validp=1;//强制校准完成，整个if就不用看了
	#endif 
		/*一直向前运动*/
//		ChassisSpeed->SetSpeedY=0.2;
		//ChassisSpeed->SetSpeedX=ChassisSpeed->SpeedError=ChassisSpeed->Spin=0;
		PositionCount++;
		if (PositionCount<50)
		{
			Position_LL.init_x+=Position_LL.x*0.02;
			Position_LL.init_y+=Position_LL.y*0.02;
		}
		else if (PositionCount>3000&&PositionCount<3050)
		{
			Position_LL.init_x-=Position_LL.x*0.02;
			Position_LL.init_y-=Position_LL.y*0.02;
		}
		else if (PositionCount>3050)
		{
			PX=Position_LL.init_x;
			PY=Position_LL.init_y;
			if (PX<PX_MIN&&PY<PY_MIN)
			{
				theta=atan(PX/PY)/6.28;
				if (PY<0)theta+=3.14;
				yaw_Base=yaw_OFFSET-theta;
				Position_LL.validp=1;
				PositionCount=0;
			}
			else Position_LL.init_x=Position_LL.init_y=PositionCount=0;
		}
	}
	else if (Position_LL.Validation&&Position_LL.validp)//状态正常
	{
		PositionCaculate();	
		PositionStruct.status=SELF_ID;//TODO:改为自己的ID
	}
	else if(RemoteLostCount) //遥控器丢数据
	{
		PositionCount=0;
		//PositionStruct.status=0;//发送未完成信号
		Position_LL.validp=0;//重新开始校准
	}
	else //定位不准
	{
		PositionCount=0;
		//PositionStruct.status=0;//发送未完成信号
		Position_LL.validp=0;//重新开始校准
		Position_LL.init_x=Position_LL.init_y=PositionCount=0;
	}
}
void PositionCaculate(void)
{
		/*计算角度*/
	if (VisionData.error_x)
		PositionStruct.expect_x=VisionData.error_x;
//	else 
//		PositionStruct.expect_x=1.21;//=UWBData.x
	if (VisionData.error_y)
		PositionStruct.expect_y=VisionData.error_y;
//	else 
//		PositionStruct.expect_y=1.5;//=UWBData.y
	/**/
	
	PositionStruct.actual_x=Position_LL.x;	
	PositionStruct.actual_y=Position_LL.y;
		/*位置移动*/
	PositionPID[0].Ref=PositionStruct.expect_x;
	PositionPID[0].Fdb=PositionStruct.actual_x;//获得当前位置
	PositionPID[0].calc(&PositionPID[0]);


	PositionPID[1].Ref=PositionStruct.expect_y;
	PositionPID[1].Fdb=PositionStruct.actual_y;
	PositionPID[1].calc(&PositionPID[1]);
	
	PositionStruct.yaw_error=(GetYawLocation()-yaw_Base)*2*PIE;//偏离正北方向
	PositionStruct.speedx=PositionPID[0].Out*cos(PositionStruct.yaw_error)-PositionPID[1].Out*sin(PositionStruct.yaw_error);//向右为正
	PositionStruct.speedy=PositionPID[1].Out*cos(PositionStruct.yaw_error)+PositionPID[0].Out*sin(PositionStruct.yaw_error);//向左为正
	
}
void Recalibrate(void)//遥控器左侧拨杆到中间再拨回，重新校准
{
#if AUTO_CALIBRATE //自动校准
		PositionCount=0;
		PositionStruct.status=0;//发送未完成信号
		UWBData.validp=0;//重新开始校准
		UWBData.init_x=UWBData.init_y=PositionCount=0;
#else 
	yaw_Base=GetYawLocation();
	Position_LL.validp=0;
#endif
}