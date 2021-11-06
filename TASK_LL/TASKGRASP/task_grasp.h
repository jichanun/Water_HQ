#ifndef __TASK_GRASP_H__
#define __TASK_GRASP_H__
#include "sys.h"
/************8
本文档同时集成了单片机向控制台发送的信号



*********************/
typedef union 
{
		u8 ConsoleSendBuf[10];
  struct
  {
	  u8 id ;
	  float expectx;
	  float expecty;
	  u8 status;
  }vars;

}ConsoleBufUnion;













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



/*********������˵��
1��    ����Ͷ�Ƕȶ��
7��    ����ͷ��ת���
3�� 	 ��е����ֱ�ƶ�
4�� 	 ��е��pitch
5�� 	 ��е��roll

*******************/
#endif 