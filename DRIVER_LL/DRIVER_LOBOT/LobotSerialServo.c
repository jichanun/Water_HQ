/*******************************************************************************
* 文件名： LobotSerial.c
* 作者： 深圳乐幻索尔科技
* 日期：20170217
* LX串口舵机Demo
*******************************************************************************/
//#include "stm32f4x.h"
#include "LobotSerialServo.h"
//#include "uart.h"
//#include "usart2.h"
#include "bool.h"
#include "BusServoCtrl.h"
#include "cmsis_os.h"

#define LobotSerialWrite  uartWriteBuf

#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 将高地八位合成为十六位
//void uartWriteBuf(uint8_t *buf, uint8_t len)
//{
//	while (len--) {
//		while ((USART2->SR & 0x40) == 0);
//		USART_SendData(USART2,*buf++);
//	}
//}	
void uartWriteBuf(uint8_t*SendBuf,int len)//串口1发送字符串
{
	while (len--) {
    while(LL_USART_IsActiveFlag_TC(USART2)!=1);//等待发送完成
    LL_USART_TransmitData8(USART2,(uint8_t)(*SendBuf & (uint16_t)0x1ff));//发送数据
    SendBuf++;
	}

}

uint8_t LobotRxBuf[16];
	
uint8_t LobotCheckSum(uint8_t buf[])
{
  uint8_t i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (uint8_t)temp;
  return i;
}

void LobotSerialServoSetID(uint8_t oldID, uint8_t newID)
{
	uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  LobotSerialWrite(buf, 7);
}

void LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time)
{
  uint8_t buff[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
	position = 1000;
  buff[0] = buff[1] = LOBOT_SERVO_FRAME_HEADER;
  buff[2] = id;
  buff[3] = 7;
  buff[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buff[5] = GET_LOW_BYTE(position);
  buff[6] = GET_HIGH_BYTE(position);
  buff[7] = GET_LOW_BYTE(time);
  buff[8] = GET_HIGH_BYTE(time);
  buff[9] = LobotCheckSum(buff);
  LobotSerialWrite(buff, 10);
}
void Grasp(int16_t P1,int16_t P2,int16_t P3,int16_t P4,int16_t P5,int16_t time)
{
	LobotSerialServoMove(1,P1,time);
	LobotSerialServoMove(2,P2,time);
	LobotSerialServoMove(3,P3,time);
	LobotSerialServoMove(4,P4,time);
	LobotSerialServoMove(5,P5,time);
//	LobotSerialServoMove(6,P6,time);
					osDelay(time+50);

}
void NewMove(u8 num, u16 time, u16 p1, u16 p2, u16 p3, u16 p4, u16 p5, u16 p6, u16 p7, u16 p8, u16 p9
							, u16 p10, u16 p11, u16 p12, u16 p13, u16 p14, u16 p15, u16 p16)
//角度0-4095
{
	u8 buff[100];
	buff[0]=buff[1]=0XFF;
	buff[2]=0XFE;
	buff[3]=num*5+4;
	buff[4]=0x83;
	buff[5]=0x2a;
	buff[6]=0x04;
	
	buff[7]=0x01;
	buff[9]=p1>>8;
	buff[8]=p1&0xff;
	buff[11]=time>>8;
	buff[10]=time&0xff;
	
	buff[12]=0x02;
	buff[14]=p2>>8;
	buff[13]=p2&0xff;
	buff[16]=time>>8;
	buff[15]=time&0xff;
	
	buff[17]=0x03;
	buff[19]=p3>>8;
	buff[18]=p3&0xff;
	buff[21]=time>>8;
	buff[20]=time&0xff;
	
	buff[22]=0x04;
	buff[24]=p4>>8;
	buff[23]=p4&0xff;
	buff[26]=time>>8;
	buff[25]=time&0xff;
	
	buff[27]=0x05;
	buff[29]=p5>>8;
	buff[28]=p5&0xff;
	buff[31]=time>>8;
	buff[30]=time&0xff;
	
	buff[32]=0x06;
	buff[34]=p6>>8;
	buff[33]=p6&0xff;
	buff[36]=time>>8;
	buff[35]=time&0xff;
	
	buff[37]=0x07;
	buff[38]=p7>>8;
	buff[39]=p7&0xff;
	buff[40]=time>>8;
	buff[41]=time&0xff;
	buff[42]=0x08;
	buff[43]=p8>>8;
	buff[44]=p8&0xff;
	buff[45]=time>>8;
	buff[46]=time&0xff;
	buff[47]=0x09;
	buff[48]=p9>>8;
	buff[49]=p9&0xff;
	buff[50]=time>>8;
	buff[51]=time&0xff;
	buff[52]=0x0A;
	buff[53]=p10>>8;
	buff[54]=p10&0xff;
	buff[55]=time>>8;
	buff[56]=time&0xff;
	buff[57]=0x0B;
	buff[58]=p11>>8;
	buff[59]=p11&0xff;
	buff[60]=time>>8;
	buff[61]=time&0xff;
	buff[62]=0x0C;
	buff[63]=p12>>8;
	buff[64]=p12&0xff;
	buff[65]=time>>8;
	buff[66]=time&0xff;
	buff[67]=0x0D;
	buff[68]=p13>>8;
	buff[69]=p13&0xff;
	buff[70]=time>>8;
	buff[71]=time&0xff;
	buff[72]=0x0E;
	buff[73]=p14>>8;
	buff[74]=p14&0xff;
	buff[75]=time>>8;
	buff[76]=time&0xff;
	buff[77]=0x0F;
	buff[78]=p15>>8;
	buff[79]=p15&0xff;
	buff[80]=time>>8;
	buff[81]=time&0xff;
	buff[82]=0x10;
	buff[83]=p16>>8;
	buff[84]=p16&0xff;
	buff[85]=time>>8;
	buff[86]=time&0xff;
	u32 sum=0;
	for(int i =2;i<87;i++)
		sum+=buff[i];
	buff[87]=~sum;
	  LobotSerialWrite(buff, 88);

}
void ReadTorque(u8 id)
{
	u8 buff[10];
	buff[0]=buff[1]=0xff;
	buff[2]=id;
	buff[3]=0x04;
	buff[4]=0x02;
	buff[5]=0x45;
	buff[6]=0x02;
	u32 sum=0;
	for (int i=2;i<7;i++)
		sum+=buff[i];
	buff[7]=~sum;
	  LobotSerialWrite(buff, 8);
}
void ReadAngle(u8 id)
{
	u8 buff[10];
	buff[0]=buff[1]=0xff;
	buff[2]=id;
	buff[3]=0x04;
	buff[4]=0x02;
	buff[5]=0x38;
	buff[6]=0x08;
	u32 sum=0;
	for (int i=2;i<7;i++)
		sum+=buff[i];
	buff[7]=~sum;
	  LobotSerialWrite(buff, 8);
}

void ReadTemperature(u8 id)
{
	u8 buff1[10];
	u32 sum=0;
	buff1[0]=buff1[1]=0xff;
	buff1[2]=id;
	buff1[3]=0x04;
	buff1[4]=0x02;
	buff1[5]=0x3f;
	buff1[6]=0x01;
	for (int i=2;i<7;i++)
		sum+=buff1[i];
	
	buff1[7]=~sum;
	LobotSerialWrite(buff1, 8);
  
}
void robstest(void)//位置和时间都是先低位后高位
{
  u8 buff[23];u32 sum=0;
	buff[0]=0xff;
	buff[1]=0xff;
	buff[2]=0xfe;
	buff[3]=0x13;
	buff[4]=0x83;
	buff[5]=0x2a;
	buff[6]=0x04;
	buff[7]=0x00;
	buff[8]=0x07;
	buff[9]=0xff;
	buff[10]=0x07;
	buff[11]=0xd0;
	buff[12]=0x01;
	buff[13]=0x00;//位置14 13
	buff[14]=0x01;
	buff[15]=0x00;//时间 16 15
	buff[16]=0x09;
	buff[17]=0x04;
	buff[18]=0x00;
	buff[19]=0x00;
	buff[20]=0x0f;
	buff[21]=0xa0;
	for (int i =2;i<22;i++)
		sum+=buff[i];
		
	buff[22]=~sum;
	  LobotSerialWrite(buff, 23);
}
void LobotSerialServoUnload(uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  LobotSerialWrite(buf, 7);
}

void LobotSerialServoLoad(uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  LobotSerialWrite(buf, 7);
}

int LobotSerialServoReadPosition(uint8_t id)
{
  int ret;
  uint8_t buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);
	
	LobotSerialWrite(buf, 6);
	
	ret = LobotSerialMsgHandle();
  return ret;
}
bool isUartRxCompleted = false;

bool isRxCompleted(void)
{
	if(isUartRxCompleted == true){
		isUartRxCompleted = false;
		return true;
	}else{
		return false;
	}
}


int LobotSerialMsgHandle(void)
{
	int count = 50000;
	uint8_t cmd;
	int ret;
	
	while(!isRxCompleted())
	{
		count--;
		if(count < 0)
			return -2048;
	}
	
	if(LobotCheckSum(LobotRxBuf) != LobotRxBuf[LobotRxBuf[3]+2])
	{
		return -2049;
	}
	
	cmd = LobotRxBuf[4];
	switch(cmd)
	{
		case LOBOT_SERVO_POS_READ:
			ret = (int)BYTE_TO_HW(LobotRxBuf[6], LobotRxBuf[5]);
			return ret;
		default:
			break;
	}
	return 0;
}
void USART2SendDataPacket(u8 txd[],u32 count)
{
	u32 i;
	for(i = 0; i < count; i++)
	{
		while((USART2->SR&0X40)==0);//循环发送,直到发送完毕
		USART2->DR = txd[i];
		while((USART2->SR&0X40)==0);//循环发送,直到发送完毕
	}
}
void BusServoCtrl(u8 id,u8 cmd,u16 prm1,u16 prm2)
{
	u32 i;
	u8 tx[20]={0};
	u8 datalLen = 4;
	u32 checkSum = 0;

	switch(cmd)
	{
	case SERVO_MOVE_TIME_WRITE:
		datalLen = SERVO_MOVE_TIME_DATA_LEN;
		break;
		
	
	}
	tx[0] = 0x55;
	tx[1] = 0x55;
	tx[2] = id;
	tx[3] = datalLen;
	tx[4] = cmd;
	tx[5] = prm1;
	tx[6] = prm1 >> 8;
	tx[7] = prm2;
	tx[8] = prm2 >> 8;
	for(i = 2; i <= datalLen + 1; i++)
	{
		checkSum += tx[i];
	}
	tx[datalLen + 2] = ~checkSum;
	//UART_TX_ENABLE();
	USART2SendDataPacket(tx,datalLen + 3);
}

