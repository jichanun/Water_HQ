#ifndef __TASK_UWB_H
#define __TASK_UWB_H	 

#include "main.h"
#include <string.h>
#include "stdlib.h"
#include "sys.h"
#include "uwb.h" 

typedef struct
{
	API_Position Tag;
	float x;
	float y;
	float z;
	u8 id;
	u8 Validation;
	u16 Info_Length;
	u8 Info_Sender;
	u8 validp;//North init Validation
	float init_x;
	float init_y;
}API_Struct;


void GetUwbData(void);




#endif