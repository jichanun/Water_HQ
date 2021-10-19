#include "uwb.h" 
#include "task_uwb.h" 

API_Struct Position_LL;
extern u8 UART2BUFF[130];
u8 TagInfoData[128];
void GetUwbData()
{
	Uwb_Get_Data(UART2BUFF,&Position_LL.Tag,&Position_LL.x,&Position_LL.y
	,&Position_LL.z,&Position_LL.Validation,TagInfoData,&Position_LL.Info_Length,
	&Position_LL.Info_Sender);
}