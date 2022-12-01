#include "uwb.h" 
#include "task_uwb.h" 
#include "driver_chassis.h"
#include "task_grasp.h"
API_Struct Position_LL;
extern u8 UART2BUFF[1000];
u8 TagInfoData[128];
u8 AnchorTransferData[1000];//????

extern PositionDataStruct PositionStruct;
extern u8 SELF_ID;
float PFilterK=0.1;

u16 Trans_Length;
uint8_t bsp_usart2_send(uint8_t *data,uint32_t len)
{
	while (LL_USART_IsActiveFlag_TC(USART2) == 0){
	}
	LL_DMA_ClearFlag_TC6(DMA1);
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_6);
	LL_DMA_SetDataLength(DMA1,LL_DMA_STREAM_6,len);
	LL_DMA_EnableStream( DMA1,LL_DMA_STREAM_6);
	return 0;
}


ConsoleBufUnion ConsoleBufRx;
ToRosUnion ROSData;
void TagUnpack(void)
{
	
	int a =0;
	PositionStruct.expect_x=ROSData.vars.px[SELF_ID];
	PositionStruct.expect_y=ROSData.vars.py[SELF_ID];
	if (ConsoleBufRx.vars.start==0xcb)
	{
		a=1;
		if (ConsoleBufRx.vars.mode==0xf0)
		{
			a=2;
			if(Position_LL.Info_Sender==0x10){
				a=5;
				if (ConsoleBufRx.vars.receiver==0x04)
					a=3;
			}
		}
	}
}
u8 AnchorSend[68];
u32 uwbtime;
void GetUwbData()
{
	switch(
	Uwb_Get_Data(UART2BUFF,&Position_LL.Tag,&Position_LL.id,&Position_LL.x,&Position_LL.y
	,&Position_LL.z,&Position_LL.Validation,TagInfoData,&Position_LL.Info_Length,
	&Position_LL.Info_Sender)){
	
		case 1:{
					
					#if 1 //发送
						AnchorSend[0]=65;//长度
						AnchorSend[1]=3;//功能勿动
					
						uwbtime= HAL_GetTick();
						AnchorSend[2]=(uwbtime>>24)&0xff;
						AnchorSend[3]=(uwbtime>>16)&0xff;
						AnchorSend[4]=(uwbtime>>8)&0xff;
						AnchorSend[5]=(uwbtime)&0xff;
						//位置↓
						AnchorSend[6]=((s16)(Position_LL.x*1000)>>8)&0xff;
					  AnchorSend[7]=((s16)(Position_LL.x*1000))&0xff;
					  AnchorSend[8]=((s16)(Position_LL.y*1000)>>8)&0xff;
					  AnchorSend[9]=((s16)(Position_LL.y*1000))&0xff;
					
					
						for (int i =10;i<67;i++)
							AnchorSend[i]='a';
						Uwb_BroadCast(AnchorSend,AnchorTransferData,&Trans_Length);
							bsp_usart2_send(AnchorTransferData,Trans_Length);

					#endif
		}
		case 2:{
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
		}break;
		case 3:
		{
			for (int i =0; i<65;i++)
				ROSData.buf[i]=TagInfoData[i];
			
			//TagUnpack();//no need for this ,in driver.c 's test module.
		}
			break;
		default:break;
		
	}
}

float ReturnX(void)
{
	return Position_LL.x;
}float ReturnY(void)
{
	return Position_LL.y;
}float ReturnZ(void)
{
	return Position_LL.z;
}
