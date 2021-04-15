#ifndef __TASK_GRASP_H__
#define __TASK_GRASP_H__
#include "sys.h"

void GraspControlTask();
void LServoInit();
typedef struct
{
	int16_t Posi;
	int16_t time;
}LobotServoStruct;
typedef struct
{
	LobotServoStruct Servo[9];
	int16_t time;
}LobotServoData;



/*********舵机编号说明
1号    事项投角度舵机
7号    摄像头旋转舵机
3号 	 机械臂竖直移动
4号 	 机械臂pitch
5号 	 机械臂roll

*******************/
#endif 