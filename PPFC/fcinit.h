#ifndef _FORMATION_CONTROL_INIT
#define _FORMATION_CONTROL_INIT
#include "stm32f4xx.h"

void FC_INIT(void);

extern double m;
extern double Ixx;
extern double Iyy;
extern double Izz;
extern double lhz;
extern double lvy;
extern double lvx;
extern double lmx;
extern double lh;
extern double Xu;
extern double Xdu;
extern double Yv;
extern double Ydv;
extern double Zw;
extern double Zdw;
extern double Pp;
extern double Pdp;
extern double Qq;
extern double Qdq;
extern double Rr;
extern double Rdr;

extern double Xn;
extern double Up;

#endif
