/****************************************************
*			Title:		ÔÆÌ¨¿ØÖÆ
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#ifndef __TASK_GIMBAL_H__
#define __TASK_GIMBAL_H__
#include "sys.h"
void YawSetLocationValueChange(float Yaw);
void PitchSetLocationValueChange(float Pitch);
void StraightLineMotorInit(void);
void StraightLineMotorControl(void);
void VisionControl(void);
void VisionTransmit(void);

typedef union
{
	u8 buf[57];
	struct{
		float data0;
		float data1;
		float px[6];
		float py[6];
		u8 status;
	}vars;
}ToRosUnion;

#endif
