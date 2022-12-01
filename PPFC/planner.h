#ifndef _BSRRTSTAR
#define _BSRRTSTAR
#include "stm32f4xx.h"
#include "string.h"
#include "sys.h"
#include "math.h"
#include "stdbool.h"
#include "time.h"
#include "stdlib.h"

#define MS 3
#define ML 10
#define MW 10
#define MR MS*ML+2
#define MC MS*MW+2
#define MAXTREESIZE 100
#define Rmin 2*MS
#define MinEdgeLength Rmin/2
#define MaxEdgeLength 3*Rmin/2
#define RewireLength MaxEdgeLength

struct TreeNode{
	u16 fNodeNumber;
	u16 node[2];
	double cost;
	double bLength;
	double cLength;
};

extern u16 MAP[MR][MC];  //
extern u16 PATH[20][2];
extern u16 PATHLENGTH;
extern double DIST[MAXTREESIZE+4];
extern u16 DISTQUEUE[MAXTREESIZE+4];
extern u16 PPFlag;

void BSRRTStar(void);
void RandNewNode(void);
void ExtendNode(void);
void ChooseFather(void);
void Rewire(void);
bool ArriveEnd(void);
bool MaxRadiusofCurvatureRestrain(u16 *ffNode,u16 *fNode,u16 *cNode);
bool NotCollision(u16 *ffNode,u16 *fNode,u16 *cNode);
void Bsplines(u16* B,u16 ffNode, u16 fNode, u16 cNode, u16 bl);
void Lines(u16* B,u16 fNode, u16 cNode, u16 bl);
void CostCalculation(void);
double getCurveLength(u16 *ffNode,u16 *fNode,u16 *cNode);
void BSRRTStar_CLEAR(void);
void BSRRTStar_INIT(void);

void setNode(void);
void setSNode(u16 n);
void setINode(u16 n, u16 *node, u16 fNodeNumber, double cost, double bLength, double cLength);
void clearNode(void);
void updateTNode(u16 fNode, double cost, double bLenght, double cLength);
double getLength(u16 *nodea, u16 *nodeb);
void getDist(void);

void setPath(void);

u16 find(u16 fNode, u16 cNode);
#endif
