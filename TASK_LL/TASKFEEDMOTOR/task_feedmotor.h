/****************************************************
*			Title:		TaskFeedMotor
*			ChipType:	STM32F405RGT6
*			Version:	1.2.3
*			Date:			2017.09.23
*												LD.
*****************************************************/

#ifndef __TASKFEEDMOTOR_H
#define __TASKFEEDMOTOR_H	 
#include "sys.h"

typedef struct 
{
	int initcode;
	int firecode;
}TriggerStruct;

void FeedMotorControlLogic(void);
void LockedMotorDetectionAndProcessed(void);
void TriggerInit(void);
void TriggerControl(void);

#endif
