#include "lqr.h"
#include "model.h"
#include "mat.h"
#include "BSPConfig.h"

#define MAXOUT 800
#define MINOUT 450

extern int RemoteLostCount;
extern u8 SELF_ID;
u16 Cspeed0=0,Cspeed1=0,Cspeed2=0,Cspeed3=0;
extern u8 AutomaticAiming;
float ChassisK=210;	
float TX[12];

float LQRK[7][12] = {0};
float U[7] = {0};
float SP = 0;

float Q[12][12] = {{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1}};

float R[7][7] = {0};
									
float P[12][12] = {0};

float BRB[12][12] = {0};

float Uk=0.3;
void getU(float* Xr, float* Ur){
	u16 i, j;
	MatI((float **)R, 7, 0.3);
	getModel(Xr, Ur);
	
	getP();
	
	getK();
	
	Xr[0] = 5;
	Xr[1] = 0;
	Xr[2] = 5;
	Xr[3] = 0;
	Xr[4] = -3;
	Xr[5] = 0;
	Xr[6] = 0;
	Xr[7] = 0;
	Xr[8] = 0;
	Xr[9] = 0;
	Xr[10] = 0;
	Xr[11] = 0;
	
	TX[0] = X[0];
	TX[1] = X[1];
	TX[2] = X[2];
	TX[3] = X[3];
	TX[4] = X[4];
	TX[5] = X[5];
	TX[6] = X[6];
	TX[7] = X[7];
	TX[8] = X[8];
	TX[9] = X[9];
	TX[10] = X[10];
	TX[11] = X[11];
	 
	while(TX[10]>PI) TX[10] -= 2*PI;
	while(TX[10]<-PI) TX[10] += 2*PI;
	
	for(i=0; i<7; i++){
		U[i] = 0;
		for(j=0; j<12; j++){
			U[i] -= LQRK[i][j]*(TX[j]-Xr[j])/2;
			
		}
	}
	
	SP += 0.1;
	
	U[0] = -0.25-0.25*sin(SP);
	U[1] = 0.25+0.25*sin(SP);
	U[2] = 0.25+0.25*sin(SP);
	U[3] = -0.25-0.25*sin(SP);
	
	/*Control*/
	if (AutomaticAiming)
	{
				#if 1 //启用关控保护
				if (!RemoteLostCount)
				{
					LL_TIM_OC_SetCompareCH1(TIM2,MIDDLE_PWM);
					LL_TIM_OC_SetCompareCH2(TIM2,MIDDLE_PWM);
					LL_TIM_OC_SetCompareCH3(TIM8,MIDDLE_PWM);
					LL_TIM_OC_SetCompareCH4(TIM8,MIDDLE_PWM);
				}	
				else 
				{		
					if (SELF_ID==3){
						Cspeed0=(s16)(-(U[3]*ChassisK)+MIDDLE_PWM);
						Cspeed1=(s16)(-(U[2]*ChassisK)+MIDDLE_PWM);
						Cspeed2=(s16)(-(U[1]*ChassisK)+MIDDLE_PWM);
						Cspeed3=(s16)(-(U[0]*ChassisK)+MIDDLE_PWM);
					}
					else if ( SELF_ID==4||SELF_ID==5){
						Cspeed0=(s16)((U[2]*ChassisK)+MIDDLE_PWM);
						Cspeed1=(s16)((U[3]*ChassisK)+MIDDLE_PWM);
						Cspeed2=(s16)(-(U[0]*ChassisK)+MIDDLE_PWM);
						Cspeed3=(s16)(-(U[1]*ChassisK)+MIDDLE_PWM);
					}
					else  //防止没收到数据
					{
						Cspeed0=(s16)((U[2]*ChassisK)+MIDDLE_PWM);
						Cspeed1=(s16)((U[3]*ChassisK)+MIDDLE_PWM);
						Cspeed2=(s16)(-(U[1]*ChassisK)+MIDDLE_PWM);
						Cspeed3=(s16)(-(U[0]*ChassisK)+MIDDLE_PWM);
////						Cspeed0=(s16)MIDDLE_PWM;
//						Cspeed1=(s16)MIDDLE_PWM;
//						Cspeed2=(s16)MIDDLE_PWM;
//						Cspeed3=(s16)MIDDLE_PWM;
				}
					
				if (Cspeed0>MAXOUT)
					Cspeed0=MAXOUT;
				else if (Cspeed0<MINOUT)
					Cspeed0=MINOUT;
				if (Cspeed1>MAXOUT)
					Cspeed1=MAXOUT;
				else if (Cspeed1<MINOUT)
					Cspeed1=MINOUT;
				if (Cspeed2>MAXOUT)
					Cspeed2=MAXOUT;
				else if (Cspeed2<MINOUT)
					Cspeed2=MINOUT;
				if (Cspeed3>MAXOUT)
					Cspeed3=MAXOUT;
				else if (Cspeed3<MINOUT)
					Cspeed3=MINOUT;
				
				
				
				
					LL_TIM_OC_SetCompareCH1(TIM2,Cspeed3);
					LL_TIM_OC_SetCompareCH2(TIM2,Cspeed2);
					LL_TIM_OC_SetCompareCH3(TIM8,Cspeed0);
					LL_TIM_OC_SetCompareCH4(TIM8,Cspeed1);
				}
				#else  //关闭关控保护
					Cspeed0=(u16)((ChassisMotor[0].Cspeed.SetCspeed*ChassisCspeedK)+MIDDLE_PWM);
					Cspeed1=(u16)((ChassisMotor[1].Cspeed.SetCspeed*ChassisCspeedK)+MIDDLE_PWM);
					Cspeed2=(u16)((ChassisMotor[2].Cspeed.SetCspeed*ChassisCspeedK)+MIDDLE_PWM);
					Cspeed3=(u16)((ChassisMotor[3].Cspeed.SetCspeed*ChassisCspeedK)+MIDDLE_PWM);
					LL_TIM_OC_SetCompareCH1(TIM2,Cspeed2);
					LL_TIM_OC_SetCompareCH2(TIM2,Cspeed3);
					LL_TIM_OC_SetCompareCH3(TIM8,Cspeed1);
					LL_TIM_OC_SetCompareCH4(TIM8,Cspeed0);
			#endif
	}
	
	
	
	
}

void getP(void){
	u16 i;
	MatCopy((float**)P,(float**)Q,12,12);
	updateBRB();
	for(i=0;i<10;i++)
		IterP();
}

void updateBRB(void){
	u16 i, j;
	float K127[12][7] = {0};
	for(i=0;i<12;i++)
		for(j=0;j<7;j++)
			K127[i][j] = 1/R[j][j]*Bh[i][j];
	MatMul((float**)BRB,(float**)K127,(float**)Bh,12,12,7,0,1,0);
}

void IterP(void){
	float MV[12][12] = {0};
	float K1212[12][12] = {0};
	MatI((float**)K1212,12,1);
	MatMul((float**)K1212,(float**)BRB,(float**)P,12,12,12,0,0,1);
	
	MatInv((float**)MV, (float**)K1212, 12);
	
	MatMul((float**)K1212,(float**)Ah,(float**)P,12,12,12,1,0,0);
	
	MatMul((float**)P,(float**)K1212,(float**)MV,12,12,12,0,0,0);
	
	MatMul((float**)K1212,(float**)P,(float**)Ah,12,12,12,0,0,0);
	
	MatAdd((float**)P,(float**)K1212,(float**)Q,12,12);
}

void getK(void){
	float K712[7][12] = {0};
	float K77[7][7] = {0};
	float MV[7][7] = {0};

	MatMul((float**)K712,(float**)Bh,(float**)P,7,12,12,1,0,0);
	
	MatCopy((float**)K77,(float**)R,7,7);

	MatMul((float**)K77,(float**)K712,(float**)Bh,7,7,12,0,0,1);
	
	MatInv((float**)MV, (float**)K77, 7);
	
	MatMul((float**)LQRK,(float**)MV,(float**)Bh,7,12,7,0,1,0);
	
	MatMul((float**)K712,(float**)LQRK,(float**)P,7,12,12,0,0,0);
	
	MatMul((float**)LQRK,(float**)K712,(float**)Ah,7,12,12,0,0,0);
	
	//MatCopy((float**)S, (float**)K, 7, 12);
}
 

