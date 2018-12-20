#include "bsp_can.h"
#include "driver_chassis.h"
#include "BSPConfig.h"
#include "task_lostcounter.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"

static CanTxMsgTypeDef Tx1Message;
static CanRxMsgTypeDef Rx1Message;
static CanTxMsgTypeDef Tx2Message;
static CanRxMsgTypeDef Rx2Message;

void CanInit(void)
{
  UserCan1FilterConfig();													//CAN1:底盘电机*2，拨弹电机*1
  UserCan2FilterConfig();													//CAN2:云台电机*2
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}

unsigned char UserCan1FilterConfig()
{
	CAN_FilterConfTypeDef  CAN1_FilerConf;
	hcan1.pTxMsg = &Tx1Message;
	hcan1.pRxMsg = &Rx1Message;

	CAN1_FilerConf.FilterIdHigh=CAN1_FILTER_ID_FMI2<<5;     //32位ID
    CAN1_FilerConf.FilterIdLow=CAN1_FILTER_ID_FMI0<<5;
    CAN1_FilerConf.FilterMaskIdHigh=CAN1_FILTER_ID_FMI3<<5; //32位MASK
    CAN1_FilerConf.FilterMaskIdLow=CAN1_FILTER_ID_FMI1<<5;
    CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
    CAN1_FilerConf.FilterNumber=0;          //过滤器0
    CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDLIST;
    CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_16BIT;
    CAN1_FilerConf.FilterActivation=ENABLE; //激活滤波器0
    CAN1_FilerConf.BankNumber=14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK) return 1;//滤波器初始化

	__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);//FIFO0消息挂起中断允许.	  

	return 0;
}

unsigned char UserCan2FilterConfig()
{
	CAN_FilterConfTypeDef  CAN2_FilerConf;
	hcan2.pTxMsg = &Tx2Message;
	hcan2.pRxMsg = &Rx2Message;
	
	CAN2_FilerConf.FilterIdHigh=CAN2_FILTER_ID_FMI2<<5;     //32位ID
    CAN2_FilerConf.FilterIdLow=CAN2_FILTER_ID_FMI0<<5;
    CAN2_FilerConf.FilterMaskIdHigh=CAN2_FILTER_ID_FMI3<<5; //32位MASK
    CAN2_FilerConf.FilterMaskIdLow=CAN2_FILTER_ID_FMI1<<5;
    CAN2_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0<<5;//过滤器0关联到FIFO0
    CAN2_FilerConf.FilterNumber=14;          //过滤器0
    CAN2_FilerConf.FilterMode=CAN_FILTERMODE_IDLIST;
    CAN2_FilerConf.FilterScale=CAN_FILTERSCALE_16BIT;
    CAN2_FilerConf.FilterActivation=ENABLE; //激活滤波器0
    CAN2_FilerConf.BankNumber=14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&CAN2_FilerConf)!=HAL_OK) return 1;//滤波器初始化

	__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_FMP0);//FIFO0消息挂起中断允许.	  
	return 0;	
}




extern GimbalMotorStruct	YawMotor,PitchMotor;
extern FeedMotorStruct	 	FeedMotor;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	u8 i;
	if(_hcan==&hcan1)
	{
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
		LostCounterFeed(_hcan->pRxMsg->FMI);
  
		for(i=0;i<8;i++)
		{
			ChassisMotor[_hcan->pRxMsg->FMI].SpeedReceiveMessege[i] = _hcan->pRxMsg->Data[i];
		}
		
	}else if(_hcan==&hcan2)
	{
		__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
		switch(_hcan->pRxMsg->FMI)
		{
			case 0: 
				for(i=0;i<8;i++)
				{
					YawMotor.CANReceiveMessege[i] = _hcan->pRxMsg->Data[i];
				}
				LostCounterFeed(GIMBAL_MOTOR_YAW);
				break;
				
			case 1:
				for(i=0;i<8;i++)
				{
					PitchMotor.CANReceiveMessege[i] = _hcan->pRxMsg->Data[i];
				}
				LostCounterFeed(GIMBAL_MOTOR_PITCH);
				break;
				
			case 2:
				for(i=0;i<8;i++)
				{
					FeedMotor.ReceiveMessege[i] = _hcan->pRxMsg->Data[i];
				}
				LostCounterFeed(FEEDMOTOR_LOST_COUNT);
				break;
				
			default:
				break;
		}
	}
	
	
}

u8 CAN1_Send_Msg(u8* msg,u8 len)
{
    u16 i=0;
    hcan1.pTxMsg->StdId=0x200;        //标准标识符
    hcan1.pTxMsg->ExtId=0x12;        //扩展标识符(29位)
    hcan1.pTxMsg->IDE=CAN_ID_STD;    //使用标准帧
    hcan1.pTxMsg->RTR=CAN_RTR_DATA;  //数据帧
    hcan1.pTxMsg->DLC=len;
    for(i=0;i<len;i++)
    hcan1.pTxMsg->Data[i]=msg[i];
    if(HAL_CAN_Transmit(&hcan1,10)!=HAL_OK) return 1;     //发送
    return 0;		
}

u8 CAN2_Send_Msg(u8* msg,u8 len)
{
    u16 i=0;
    hcan2.pTxMsg->StdId=0x1FF;        //标准标识符
    hcan2.pTxMsg->ExtId=0x12;        //扩展标识符(29位)
    hcan2.pTxMsg->IDE=CAN_ID_STD;    //使用标准帧
    hcan2.pTxMsg->RTR=CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC=len;
    for(i=0;i<len;i++)
    hcan2.pTxMsg->Data[i]=msg[i];
    if(HAL_CAN_Transmit(&hcan2,10)!=HAL_OK) return 1;     //发送
    return 0;		
}