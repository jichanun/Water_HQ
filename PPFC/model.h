#ifndef _MODEL
#define _MODEL
#include "stm32f4xx.h"
#include "math.h"
#include "sys.h"
#define PI 3.1415926535

extern float Ah[12][12];
extern float Bh[12][7];
extern float T;
extern u16 PATHPOINT;
extern float X[12];
extern float Xr[12];

void getModel(float* Xr, float* Ur);
void getAr(float* Xr, float* Ur);
void getBr(float* Xr, float* Ur);
void setXr(void);
float getOri(u16 p);
void discretization(void);

#endif
