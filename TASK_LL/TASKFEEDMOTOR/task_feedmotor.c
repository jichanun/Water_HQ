#include "driver_feedmotor.h"
#include "task_feedmotor.h"
#include "math.h"
#include "task_remote.h"
#include "task_gimbal.h"

//热量控制参数
#define HEAT_UPPER_LIMIT  (480.0f)
#define COOLlNG_RATIO   	(160.0f)
#define SHOOT_SPEED     	(23.0f)
#define SPARE_HEAT        (20.0f)

//拨弹参数
#define ENCODER_LINE (36.0f)
#define SHOOT_NUMBER (8.0f)
#define SHOOT_ONCE_LOCATION ( ENCODER_LINE/SHOOT_NUMBER )

//堵转回转拨盘格数
#define MOTOR_RETURN_RATE (0.5f)

float SetLocation;

//----------------------------------------------拨弹周期任务----------------------------------------------------------
void FeedMotorControlLogic()
{	

	
//PID控制和can信号发送放在云台任务中了
}


int IsDeverseLocked=0;
void LockedMotorDetectionAndProcessed(void)		//堵转检测算法
{
	static int 		TimeCount=0;								//用来产生0.1s时段供堵转使用
	static float 	Current[10]={0,0,0,0,0,0,0,0,0,0};
	static float  CurrentMean;
	static int MotorLockedCount = 0;//堵转次数计算

	//更新电流
	for(int i=0;i<9;i++)
	{
			Current[i]=Current[i+1];
	}
	Current[9]=GetFeedMotorCurrent();
	
	//堵转检测
	if(IsDeverseLocked==1)
	{
			if(TimeCount<25)			//0.1s一个周期
			  TimeCount++; 
			else
			{
				TimeCount = 0;
				IsDeverseLocked = 0;
				SetLocation += MOTOR_RETURN_RATE * SHOOT_ONCE_LOCATION;	//正转
				SetFeedMotorSetLocation(SetLocation);
			}
	}
	else
	{	
			CurrentMean=(Current[0]+Current[1]+Current[2]+Current[3]+Current[4]+Current[5]+Current[6]+Current[7]+Current[8]+Current[9])/10.0f; 
			
			if( fabs(CurrentMean) > 6000.0f)
			{
				MotorLockedCount++;
				SetLocation = GetFeedMotorLocation() - MOTOR_RETURN_RATE * SHOOT_ONCE_LOCATION;	//反转
				SetFeedMotorSetLocation(SetLocation);
				IsDeverseLocked = 1;					
			}
			else
				MotorLockedCount = 0;
	}
	
}


void FeedMotorLocationUpdate(unsigned char ShootNumber)	//设定子弹发射个数接口
{
		if(IsDeverseLocked == 0)	//不堵转
		{
			if( fabs(GetFeedMotorLocationError()) < fabs(SHOOT_ONCE_LOCATION) )	
			{
							SetLocation += SHOOT_ONCE_LOCATION * ShootNumber;
							SetFeedMotorSetLocation(SetLocation);
			}
		}
}

void FeedMotorSingleShootSet(u8 FeedMotorJudge)		//单发函数
{
	static int SingleShootEnable=1;
	if(FeedMotorJudge&&SingleShootEnable)
	{
			SingleShootEnable = 0;			
			FeedMotorLocationUpdate(1);
	}
	else if(FeedMotorJudge==0)
	{
			SingleShootEnable = 1;
	}
}
