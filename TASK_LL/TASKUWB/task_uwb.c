#include "uwb.h" 
#include "task_uwb.h" 
#include "driver_chassis.h"

API_Struct Position_LL;
extern u8 UART2BUFF[1000];
u8 TagInfoData[128];
extern PositionDataStruct PositionStruct;
extern u8 SELF_ID;
float PFilterK=0.1;

void GetUwbData()
{
	Uwb_Get_Data(UART2BUFF,&Position_LL.Tag,&Position_LL.id,&Position_LL.x,&Position_LL.y
	,&Position_LL.z,&Position_LL.Validation,TagInfoData,&Position_LL.Info_Length,
	&Position_LL.Info_Sender);
	SELF_ID=Position_LL.id;
	for (int i =0;i<Position_LL.Tag.Present_Nodes;i++)
	{
		if (Position_LL.Tag.Tag[i].Validation)
		{
			#if 1  //Filter
				Position_LL.Tag.Tag[i].Pos_X=Position_LL.Tag.Tag[i].Pos_X*PFilterK+Position_LL.Tag.TagLast[i].Pos_X*(1-PFilterK);
				Position_LL.Tag.Tag[i].Pos_Y=Position_LL.Tag.Tag[i].Pos_Y*PFilterK+Position_LL.Tag.TagLast[i].Pos_Y*(1-PFilterK);
				Position_LL.Tag.TagLast[i].Pos_X=Position_LL.Tag.Tag[i].Pos_X;
				Position_LL.Tag.TagLast[i].Pos_Y=Position_LL.Tag.Tag[i].Pos_Y;
			#endif
			PositionStruct.Position[Position_LL.Tag.Tag[i].ID].px=Position_LL.Tag.Tag[i].Pos_X;
			PositionStruct.Position[Position_LL.Tag.Tag[i].ID].py=Position_LL.Tag.Tag[i].Pos_Y;
			Position_LL.Tag.Tag[i].Validation=0;
		}
	}
}
