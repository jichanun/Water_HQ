#include "driver_laser.h"
#include "driver_gimbal.h"
#include "cmsis_os.h"

#define Code1 71
#define Code0 34

#define NUM 8
#define LENTH (NUM*24+240)//后100个
void LaserControl(unsigned char judge)
{
	PCout(7)=judge;
}
LED_Typedef led[NUM];
u32 data[LENTH];
u32 data1[LENTH];
void LEDControl(u8 judge)
{
	if (judge=='C')
	{
		    osDelay(2000);
		for (int j =0;j<6;j++)
		{
				if (j%2)
					for(int i=0;i<NUM;i++)
				{
					led[i].r = 255;
					led[i].g = 0;
					led[i].b = 0;
				}
				else 
						for(int i=0;i<NUM;i++)
					{
						led[i].r = 0;
						led[i].g = 0;
						led[i].b = 0;
					}
				LED_setdata();
		    osDelay(200);
		}
	}
	else if (judge=='S')
	{
		    osDelay(2000);
		for (int j =0;j<6;j++)
		{
				if (j%2)
				for(int i=0;i<NUM;i++)
				{
					led[i].r = 0;
					led[i].g = 255;
					led[i].b = 0;
				}
				else 
				{
						for(int i=0;i<NUM;i++)
					{
						led[i].r = 0;
						led[i].g = 0;
						led[i].b = 0;
					}
				}
				LED_setdata();
				osDelay(200);
		}
	}
	else 
	{
			for(int i=0;i<NUM;i++)
		{
			led[i].r = 0;
			led[i].g = 0;
			led[i].b = 0;
		}
		LED_setdata();
	}
}
void LED_setdata(void)
{

	for(int i=0;i<NUM;i++)
	{
		for(int j=0;j<8;j++)
		{
			data[i*24+j]=((led[i].g)>>(7-j))&1?Code1:Code0;
			data1[i*24+j]=((led[i].g)>>(7-j))&1?Code1:Code0;
		}
		for(int j=0;j<8;j++)
		{
			data[i*24+j+8]=((led[i].b)>>(7-j))&1?Code1:Code0;
			data1[i*24+j+8]=((led[i].r)>>(7-j))&1?Code1:Code0;
		}
		for(int j=0;j<8;j++)
		{
			data[i*24+j+16]=((led[i].r)>>(7-j))&1?Code1:Code0;
			data1[i*24+j+16]=((led[i].b)>>(7-j))&1?Code1:Code0;
		}
	}
	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH1);//使能TIM1 的cc通道ch3	
    LL_TIM_EnableCounter(TIM3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
}



void LaserInit(void)
{
	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableAllOutputs(TIM3);

		for(int i=0;i<NUM;i++)
	{
		led[i].g = 0;
		led[i].r = 10;
		led[i].b = 0;
	}
	
}
