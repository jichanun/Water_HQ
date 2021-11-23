#include "uwb.h" 
#include "task_uwb.h" 
#include "task_gimbal.h"
API_Struct Position_LL;
extern u8 UART2BUFF[1000];
u8 TagInfoData[128];
u8 AnchorTransferData[1000];//????
u16 Trans_Length;

API_Position TagsPosition;
u8 Anchor_Info_Data[128];
u16 Anchor_Info_Length;
u8 Anchor_Info_Sender;
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
void GetUwbData()
{
//	Uwb_Get_Data(UART2BUFF,&Position_LL.Tag,&Position_LL.x,&Position_LL.y
//	,&Position_LL.z,&Position_LL.Validation,TagInfoData,&Position_LL.Info_Length,
//	&Position_LL.Info_Sender);
}

void GetAnchorData()
{
	if (Uwb_BroadCast(UART2BUFF,AnchorTransferData,&Trans_Length,&TagsPosition
		, Anchor_Info_Data,&Anchor_Info_Length,&Anchor_Info_Sender))
	{
		bsp_usart2_send(AnchorTransferData,Trans_Length);
	}
	
}
u8 AnchorSend[60];
extern uint8_t UsbRxBuf[60];
extern u8 RosReceiveFlag;
ToRosUnion ToAncherData;
void AnchorSendBuff(void)
{
	if (RosReceiveFlag)
	{
		for (int i =0;i<57;i++)
			ToAncherData.buf[i]=UsbRxBuf[i];
		
		AnchorSend[0]=57;//长度
		AnchorSend[1]=3;//功能勿动
		for (int i =2;i<59;i++)
			AnchorSend[i]=ToAncherData.buf[i-2];
		
		Uwb_BroadCast(AnchorSend,AnchorTransferData,&Trans_Length,&TagsPosition
				, Anchor_Info_Data,&Anchor_Info_Length,&Anchor_Info_Sender);
	}
	RosReceiveFlag=0;
}

