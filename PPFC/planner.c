#include "planner.h"
#include "quicksort.h"
#include "model.h"
//#include "usart.h"

struct TreeNode Tree[MAXTREESIZE+4];
u16 RC = MR;
u16 MAP[MR][MC] = {0}; 
u16 NodeMap[MR][MC] = {0};
u16 NodeChild[MAXTREESIZE+4][MAXTREESIZE/5] = {0};

u16 TreeSize = 0;

u16 PATH[20][2];
u16 PATHLENGTH;
u16 PPFlag = 1;

double DIST[MAXTREESIZE+4];
u16 DISTQUEUE[MAXTREESIZE+4];

u16 StartLocation[2] = {0};
u16 EndLocation[2] = {29,29};
double StartOrientation = 0;
double EndOrientation = 0;
u16 StartNode[2] = {0};
u16 EndNode[2] = {0};
u16 StartFatherNode[2] = {0};
u16 EndChildNode[2] = {0};

double TryLength[3] = {MaxEdgeLength, (MaxEdgeLength+MinEdgeLength)/2, MinEdgeLength};

float Greedy = 2;

struct TreeNode treenode;

void BSRRTStar(void){
	bool Arrived = false;
	u16 iter;
	
	if(PPFlag==0) return;
	else if(pow(X[0]-EndLocation[0],2)+pow(X[2]-EndLocation[1],2)<9){
		PPFlag = 0;
		return;
	}
	
	srand((u16) (X[0]*100+X[2]*100));
	
	BSRRTStar_CLEAR();
	
	BSRRTStar_INIT();
	
//	for(titer=0; titer<MAXTREESIZE+4; titer++){
//		printf("*%3d *%5d %5d %5d %5d %9f %9f %9f ", titer, titer, Tree[titer].fNodeNumber,Tree[titer].node[0],Tree[titer].node[1],Tree[titer].cost,Tree[titer].bLength,Tree[titer].cLength);
//	}
	
	for(iter=0; iter<MAXTREESIZE; iter++){
		do{
			if(!Arrived){
				RandNewNode();
			}else if(Greedy>rand()%100){
				treenode.node[0] = EndNode[0];
				treenode.node[1] = EndNode[1];
			}else{
				RandNewNode();
			}
			ExtendNode();
		}while(treenode.fNodeNumber==MAXTREESIZE+4);
		getDist();
		quicksort(TreeSize);
		ChooseFather();
		setNode();
		Rewire();
		if(!Arrived&&ArriveEnd()){
			NodeChild[TreeSize][0]++;
			NodeChild[TreeSize][NodeChild[TreeSize][0]] = TreeSize+1;
			
			treenode.fNodeNumber = TreeSize;
			treenode.node[0] = EndNode[0];
			treenode.node[1] = EndNode[1];
			CostCalculation();
			setNode();
			
			NodeChild[TreeSize][0]++;
			NodeChild[TreeSize][1] = MAXTREESIZE+2;
			
			treenode.fNodeNumber = TreeSize;
			treenode.node[0] = EndChildNode[0];
			treenode.node[1] = EndChildNode[1];
			CostCalculation();
			setSNode(MAXTREESIZE+2);
			
			Arrived = true;
		}
	}
}

void RandNewNode(void){
	do{
		treenode.node[0] = rand()%RC;
		treenode.node[1] = rand()%RC;
	}while(MAP[treenode.node[0]][treenode.node[1]]==1||NodeMap[treenode.node[0]][treenode.node[1]]==1);
}

void ExtendNode(void){
	u16 fNodeNumber = MAXTREESIZE+4, tryTime = 0, ffNode[2], fNode[2], cNode[2], step;
	double dist;
	
	getDist();
	
	quicksort(TreeSize);
	
	cNode[0] = treenode.node[0];
	cNode[1] = treenode.node[1];
	do{
		fNode[0] = Tree[DISTQUEUE[tryTime]].node[0];
		fNode[1] = Tree[DISTQUEUE[tryTime]].node[1];
		ffNode[0] = Tree[Tree[DISTQUEUE[tryTime]].fNodeNumber].node[0];
		ffNode[1] = Tree[Tree[DISTQUEUE[tryTime]].fNodeNumber].node[1];
		dist = DIST[tryTime];
		if(dist>MinEdgeLength){
			for(step=0;step<3;step++){
				if(dist>TryLength[step]){
					cNode[0] = ((dist-TryLength[step])*fNode[0] + TryLength[step]*cNode[0])/dist;
					cNode[1] = ((dist-TryLength[step])*fNode[1] + TryLength[step]*cNode[1])/dist;
				}
				if(MaxRadiusofCurvatureRestrain(ffNode,fNode,cNode)&&NodeMap[cNode[0]][cNode[1]]==0&&NotCollision(ffNode,fNode,cNode)){
					fNodeNumber = DISTQUEUE[tryTime];
					break;
				}
			}
		}
	}while(fNodeNumber==MAXTREESIZE+4&&++tryTime<=TreeSize);
	if(fNodeNumber!=MAXTREESIZE+4){
		treenode.fNodeNumber = fNodeNumber;
		treenode.node[0] = cNode[0];
		treenode.node[1] = cNode[1];
		CostCalculation();
	}else{
		clearNode();
	}
}

void ChooseFather(void){
	u16 nodeNumberCount = 0, *ffNode, *fNode, nN;
	double bLength, cLength, cost;
	if(DIST[TreeSize]<=RewireLength) nodeNumberCount = TreeSize+1;
	else while(DIST[nodeNumberCount]<RewireLength) nodeNumberCount++;
	if(nodeNumberCount==0) return;
	nN = 0;
	while(nN<nodeNumberCount){
		fNode = Tree[DISTQUEUE[nN]].node;
		ffNode = Tree[Tree[DISTQUEUE[nN]].fNodeNumber].node;
		bLength = getLength(fNode,treenode.node)/2;
		cLength = getCurveLength(ffNode,fNode,treenode.node);
		cost = Tree[DISTQUEUE[nN]].cost - Tree[DISTQUEUE[nN]].bLength + cLength + bLength;
		if(cost<treenode.cost){
			if(MaxRadiusofCurvatureRestrain(ffNode,fNode,treenode.node)&&NotCollision(ffNode,fNode,treenode.node)){
				updateTNode(DISTQUEUE[nN],cost,bLength,cLength);
				break;
			}
		}
		nN++;
	}
}

void Rewire(void){
	u16 nodeNumberCount = 0, *ffNode, *cNode, nN, oldFatherNodeNumber, deleteChildNodeLocationinNodeChild;
	double bLength, cLength, cost;
	if(DIST[TreeSize-1]<=RewireLength) nodeNumberCount = TreeSize;
	else while(DIST[nodeNumberCount]<RewireLength) nodeNumberCount++;
	if(nodeNumberCount==0) return;
	ffNode = Tree[treenode.fNodeNumber].node;
	nN = 0;
	while(nN<nodeNumberCount){
		cNode = Tree[DISTQUEUE[nN]].node;
		bLength = getLength(treenode.node, cNode)/2;
		cLength = getCurveLength(ffNode, treenode.node, cNode);
		cost = treenode.cost - treenode.bLength + cLength + bLength;
		if(cost < Tree[DISTQUEUE[nN]].cost){
			if(MaxRadiusofCurvatureRestrain(ffNode,treenode.node,cNode) && NotCollision(ffNode,treenode.node,cNode)){
				oldFatherNodeNumber = Tree[DISTQUEUE[nN]].fNodeNumber;
				Tree[DISTQUEUE[nN]].fNodeNumber = TreeSize;
				Tree[DISTQUEUE[nN]].cost = cost;
				Tree[DISTQUEUE[nN]].bLength = bLength;
				Tree[DISTQUEUE[nN]].cLength = cLength;
				
				deleteChildNodeLocationinNodeChild = find(oldFatherNodeNumber,DISTQUEUE[nN]);
				NodeChild[oldFatherNodeNumber][deleteChildNodeLocationinNodeChild] = NodeChild[oldFatherNodeNumber][NodeChild[oldFatherNodeNumber][0]];
				NodeChild[oldFatherNodeNumber][NodeChild[oldFatherNodeNumber][0]--] = 0;
				NodeChild[TreeSize][++NodeChild[TreeSize][0]] = DISTQUEUE[nN];
			}
		}
		nN++;
	}
}

bool ArriveEnd(void){
	if(getLength(treenode.node,EndNode)<MaxEdgeLength && MaxRadiusofCurvatureRestrain(Tree[Tree[TreeSize].fNodeNumber].node,treenode.node,EndNode) && MaxRadiusofCurvatureRestrain(treenode.node,EndNode,EndChildNode)) 
		return true;
	else
		return false;
}

bool MaxRadiusofCurvatureRestrain(u16 *ffNode,u16 *fNode,u16 *cNode){
	double L0 = pow(pow(ffNode[0]-fNode[0],2) + pow(ffNode[1]-fNode[1],2),0.5);
	double L1 = pow(pow(fNode[0]-cNode[0],2) + pow(fNode[1]-cNode[1],2),0.5);
	double cosA = -((ffNode[0]-fNode[0])*(cNode[0]-fNode[0])+(ffNode[1]-fNode[1])*(cNode[1]-fNode[1]))/L0/L1;
	double sinA = pow(1-cosA*cosA,0.5);
	double maxcurvature, rmin;
	if(cosA>L1/L0)
		maxcurvature = sinA*L0/L1/L1;
	else if(cosA>L0/L1)
		maxcurvature = sinA*L1/L0/L0;
	else
		maxcurvature = pow(L0*L0+L1*L1-2*L0*L1*cosA,3/2)/(L0*L0*L1*L1*sinA*sinA);
	rmin = 1/maxcurvature;
	if(rmin >= Rmin)
		return true;
	else
		return false;
}

bool NotCollision(u16 *ffNode,u16 *fNode,u16 *cNode){
	u16 step, LL = getLength(fNode,cNode)/2, BL = (getLength(fNode,cNode) + getLength(ffNode,cNode) + getLength(ffNode,fNode))/3;
	u16 BX[MS], BY[MS];
	BL = BL<MS?BL:MS;
	LL = LL<MS?LL:MS;
	
	Bsplines(BX,ffNode[0],fNode[0],cNode[0],BL);
	Bsplines(BY,ffNode[1],fNode[1],cNode[1],BL);
	for(step=0; step<BL; step++){
		if(MAP[BX[step]][BY[step]]==1) return false;
	}
	Lines(BX,fNode[0],cNode[0],LL);
	Lines(BY,fNode[1],cNode[1],LL);
	for(step=0;step<LL;step++){
		if(MAP[BX[step]][BY[step]]==1) return false;
	}
	return true;
}

void Bsplines(u16* B, u16 ffNode, u16 fNode, u16 cNode, u16 bl){
	u16 step;
	double T;
	for(step = 0; step<bl; step++){
		T = step/bl;
		B[step] = ((1-2*T+T*T)*ffNode+(1+2*T-2*T*T)*fNode+T*T*cNode)/2;
	}
	return;
}

void Lines(u16* B, u16 fNode, u16 cNode, u16 bl){
	u16 step;
	for(step = 0; step<bl; step++){
		B[step] = ((bl-step)*fNode + step*cNode)/bl;
	}
	return;
}

void CostCalculation(void){
	u16 *fNode = Tree[treenode.fNodeNumber].node, *ffNode = Tree[Tree[treenode.fNodeNumber].fNodeNumber].node;
	treenode.bLength = getLength(fNode,treenode.node)/2;
	treenode.cLength = getCurveLength(ffNode,fNode,treenode.node);
	treenode.cost = Tree[treenode.fNodeNumber].cost - Tree[treenode.fNodeNumber].bLength + treenode.cLength + treenode.bLength;
}

double getCurveLength(u16 *ffNode,u16 *fNode,u16 *cNode){
	double Ax = ffNode[0]-2*fNode[0]+cNode[0];
	double Bx = fNode[0]-ffNode[0];
	double Ay = ffNode[1]-2*fNode[1]+cNode[1];
	double By = fNode[1]-ffNode[1];
	double A = Ax*Ax+Ay*Ay;
	double B = 2*(Ax*Bx+Ay*By);
	double C = Bx*Bx+By*By;
	double cLength = (2*A+B)*pow(A+B+C,0.5)/(4*A)+(4*A*C-B*B)*(log((2*A+B)/(2*pow(A,0.5))+pow(A+B+C,0.5))-log(B/(2*pow(A,0.5))+pow(C,0.5)))/(8*pow(A,1.5))-B*pow(C,0.5)/(4*A);
	if(isnan(cLength))
			cLength = (getLength(ffNode,fNode)+getLength(fNode,cNode))/2;
	return cLength;
}

void BSRRTStar_CLEAR(void){
	u16 i, j;
	for(i=0;i<MR;i++)
		for(j=0;j<MC;j++)
			NodeMap[i][j] = 0;
	for(i=0;i<MAXTREESIZE+4;i++)
		for(j=0;j<MAXTREESIZE/5;j++)
			NodeChild[i][j] = 0;
	TreeSize = 0;
}

void BSRRTStar_INIT(void){
	StartLocation[0] = X[0];
	StartLocation[1] = X[2];
	StartOrientation = X[10];
	StartNode[0] = EndLocation[0]-cos(EndOrientation)*Rmin/2;
	StartNode[1] = EndLocation[1]-sin(EndOrientation)*Rmin/2;
	EndNode[0] = StartLocation[0]+cos(StartOrientation)*Rmin/2;
	EndNode[1] = StartLocation[1]+sin(StartOrientation)*Rmin/2;
	StartFatherNode[0] = EndLocation[0]+cos(EndOrientation)*Rmin/2;
	StartFatherNode[1] = EndLocation[1]+sin(EndOrientation)*Rmin/2;
	EndChildNode[0] = StartLocation[0]-cos(StartOrientation)*Rmin/2;
	EndChildNode[1] = StartLocation[1]-sin(StartOrientation)*Rmin/2;
	setINode(MAXTREESIZE+3, StartFatherNode, MAXTREESIZE+3, 0, 0, 0);
	setINode(0, StartNode, MAXTREESIZE+3, Rmin/2, Rmin/2, 0);
	NodeMap[StartNode[0]][StartNode[1]] = 1;
	NodeMap[EndNode[0]][EndNode[1]] = 1;
}

void setNode(void){
	TreeSize++;
	Tree[TreeSize].node[0] = treenode.node[0];
	Tree[TreeSize].node[1] = treenode.node[1];
	Tree[TreeSize].fNodeNumber = treenode.fNodeNumber;
	Tree[TreeSize].cost = treenode.cost;
	Tree[TreeSize].bLength = treenode.bLength;
	Tree[TreeSize].cLength = treenode.cLength;
	NodeMap[treenode.node[0]][treenode.node[1]] = 1;
	NodeChild[treenode.fNodeNumber][0]++;
	NodeChild[treenode.fNodeNumber][NodeChild[treenode.fNodeNumber][0]] = TreeSize;
}

void setSNode(u16 n){
	Tree[n].node[0] = treenode.node[0];
	Tree[n].node[1] = treenode.node[1];
	Tree[n].fNodeNumber = treenode.fNodeNumber;
	Tree[n].cost = treenode.cost;
	Tree[n].bLength = treenode.bLength;
	Tree[n].cLength = treenode.cLength;
	NodeMap[treenode.node[0]][treenode.node[1]] = 1;
}

void setINode(u16 n, u16 *node, u16 fNodeNumber, double cost, double bLength, double cLength){
	Tree[n].node[0] = node[0];
	Tree[n].node[1] = node[1];
	Tree[n].fNodeNumber = fNodeNumber;
	Tree[n].cost = cost;
	Tree[n].bLength = bLength;
	Tree[n].cLength = cLength;
	NodeMap[node[0]][node[1]] = 1;
}

void clearNode(void){
	treenode.node[0] = 0;
	treenode.node[1] = 0;
	treenode.fNodeNumber = MAXTREESIZE+4;
	treenode.cost = 0;
	treenode.bLength = 0;
	treenode.cLength = 0;
}

void updateTNode(u16 fNode, double cost, double bLenght, double cLength){
	treenode.fNodeNumber = fNode;
	treenode.cost = cost;
	treenode.bLength = bLenght;
	treenode.cLength = cLength;
}

double getLength(u16 *nodea, u16 *nodeb){
	return pow(pow(nodea[0]-nodeb[0],2) + pow(nodea[1]-nodeb[1],2),0.5);
}

void getDist(void){
	u16 step;
	for(step=0; step<=TreeSize; step++){
		DIST[step] = pow(pow(Tree[step].node[0]-treenode.node[0],2) + pow(Tree[step].node[1]-treenode.node[1],2),0.5);
	}
	return;
}

u16 find(u16 fNode, u16 cNode){
	u16 cl = NodeChild[fNode][0], k;
	for(k=1;k<=cl;k++){
		if(NodeChild[fNode][k]==cNode) return k;
	}
	return -1;
}

void setPath(void){
	u16 np = MAXTREESIZE+2;
	if(PPFlag==0) return;
	PATHLENGTH = 0;
	while(np!=Tree[np].fNodeNumber){
		PATH[PATHLENGTH][0] = Tree[np].node[0];
		PATH[PATHLENGTH++][1] = Tree[np].node[1];
		np = Tree[np].fNodeNumber;
	}
	PATH[PATHLENGTH][0] = EndLocation[0];
	PATH[PATHLENGTH++][1] = EndLocation[1];
	PATHPOINT = 1;
}
