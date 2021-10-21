#include "uwb.h" 
#include "task_uwb.h" 
#include "driver_chassis.h"

API_Struct Position_LL;
extern u8 UART2BUFF[1000];
u8 TagInfoData[128];
extern PositionDataStruct PositionStruct;

void GetUwbData()
{
	Uwb_Get_Data(UART2BUFF,&Position_LL.Tag,&Position_LL.x,&Position_LL.y
	,&Position_LL.z,&Position_LL.Validation,TagInfoData,&Position_LL.Info_Length,
	&Position_LL.Info_Sender);
	for (int i =0;i<Position_LL.Tag.Present_Nodes;i++)
	{
		if (Position_LL.Tag.Tag[i].Validation)
		{
			PositionStruct.Position[Position_LL.Tag.Tag[i].ID].px=Position_LL.Tag.Tag[i].Pos_X;
			PositionStruct.Position[Position_LL.Tag.Tag[i].ID].py=Position_LL.Tag.Tag[i].Pos_Y;
			Position_LL.Tag.Tag[i].Validation=0;
		}
	}
}