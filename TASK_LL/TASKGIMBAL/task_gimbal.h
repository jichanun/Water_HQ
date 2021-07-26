/****************************************************
*			Title:		ÔÆÌ¨¿ØÖÆ
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#ifndef __TASK_GIMBAL_H__
#define __TASK_GIMBAL_H__
void YawSetLocationValueChange(float Yaw);
void PitchSetLocationValueChange(float Pitch);
void StraightLineMotorInit(void);
void StraightLineMotorControl(void);
void VisionControl(void);
void VisionTransmit(void);

#endif
