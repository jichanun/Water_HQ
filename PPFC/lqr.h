#ifndef _LQR
#define _LQR
#include "stm32f4xx.h"
#include "string.h"

extern float K[7][12];
extern float U[7];
extern float P[12][12];
extern float TX[12];

void getU(float* Xr, float* Ur);
void getP(void);
void updateBRB(void);
void IterP(void);
void getK(void);

#endif
