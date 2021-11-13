#include "task_grasp.h"
#include "LobotSerialServo.h"

LobotServoData LServo;
void LServoInit()
{
	for (int i =0;i<9;i++)
	{
		LServo.Servo[i].Posi=500;
		LServo.time=1000;
	}
		LServo.Servo[5].Posi=270;
}
/*********舵机编号说明
1号    事项投角度舵机
7号    摄像头旋转舵机
3号 	 机械臂竖直移动
4号 	 机械臂pitch
5号 	 机械臂roll

*******************/
u8 SendAncher[22];
ConsoleBufUnion ConsoleBuf;
void ConsoleSend()
{
	
	ConsoleBuf.vars.start=0xcb;
	ConsoleBuf.vars.mode=0xf0;
	ConsoleBuf.vars.sender=0x10;
	ConsoleBuf.vars.receiver=0x04;
	ConsoleBuf.vars.x1=0.1;
	ConsoleBuf.vars.x2=0.2;
	ConsoleBuf.vars.expectx=0.3;
	ConsoleBuf.vars.expecty=0.4;
	ConsoleBuf.vars.status=5;
	for (int i=0;i<sizeof(SendAncher);i++)
		SendAncher[i]=ConsoleBuf.ConsoleSendBuf[i];
	uartWriteBuf(SendAncher,sizeof(SendAncher));
}
void GraspControlTask()
{
			//Grasp(LServo[1].Posi,LServo[2].Posi,LServo[3].Posi,LServo[4].Posi,LServo[5].Posi,1000);
//	LobotSerialServoMove(6,P6,time);
		//Grasp(800,500,500,500,500,1000);
//	for (int i=2 ;i<9;i++)
//	{
//		LobotSerialServoMove(i,LServo.Servo[i].Posi,LServo.time);
//		
//	}
	ConsoleSend();
}