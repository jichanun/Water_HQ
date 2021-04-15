/****************************************************
*			Title:		Ä¦²ÁÂÖ
*			Version:	1.0.0
*			Data:			2016.12.06
*												LD.
*****************************************************/
#ifndef __LAZER_H__
#define __LAZER_H__
#include "sys.h"
#include "stm32f4xx.h"


void LaserControl(unsigned char judge);
void LEDControl(u8 judge);
typedef struct
{                   
	char g;
	char r;
	char b;
}LED_Typedef;
void LED_setdata(void);
void LaserInit(void);

#endif
