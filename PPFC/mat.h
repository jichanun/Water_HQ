#ifndef _MAT
#define _MAT
#include "stm32f4xx.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "sys.h"
void MatInv(float** MatInv, float** Mat, u16 rc);
void MatCopy(float** MatA, float** MatB, u16 r, u16 c);
void MatI(float** Mat, u16 rc, float k);
void MatMul(float** Mat, float** MatA, float** MatB, u16 I, u16 J, u16 K, u16 SA, u16 SB, u16 SC);
void MatAdd(float** Mat, float** MatA, float** MatB, u16 R, u16 C);

#endif
