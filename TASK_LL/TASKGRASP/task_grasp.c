#include "task_grasp.h"
#include "LobotSerialServo.h"

LobotServoData LServo;
void LServoInit()
{
	for (int i =0;i<9;i++)
	{
		LServo.Servo[i].Posi=500;
		LServo.time=5000;
	}
}
/*********舵机编号说明
1号    事项投角度舵机
7号    摄像头旋转舵机
3号 	 机械臂竖直移动
4号 	 机械臂pitch
5号 	 机械臂roll

*******************/
void GraspControlTask()
{
			//Grasp(LServo[1].Posi,LServo[2].Posi,LServo[3].Posi,LServo[4].Posi,LServo[5].Posi,1000);
//	LobotSerialServoMove(6,P6,time);
		//Grasp(800,500,500,500,500,1000);
	for (int i=2 ;i<9;i++)
	{
		LobotSerialServoMove(i,LServo.Servo[i].Posi,LServo.time);
		
	}
}