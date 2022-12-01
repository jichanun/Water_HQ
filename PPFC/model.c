#include "model.h"
#include "planner.h"
#include "fcinit.h"

float Ar[12][12] = {0};
float Br[12][7] = {0};
float Ah[12][12] = {0};
float Bh[12][7] = {0};
float T = 1;
u16 PATHPOINT;
float X[12];
float Xr[12];

float I[12][12] = {{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
										{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};

void getModel(float* Xr, float* Ur){
	getAr(Xr, Ur);
	getBr(Xr, Ur);
	discretization();
}
										
void getAr(float* Xr, float* Ur){
	Ar[0][1] = 1;
	
	Ar[1][1] = (Xu*cos(Xr[10])*(m-Ydv*cos(Xr[10])))/m/m;
	Ar[1][3] = (Xu*sin(Xr[10])*(m-Ydv*cos(Xr[10])))/m/m;
	Ar[1][10] = (1.414*Xu*Ydv*(Xr[1]*sin(2*Xr[10])-Xr[3]*cos(2*Xr[10]))+1.414*m*Xu*(Xr[3]*cos(Xr[10])-Xr[1]*sin(Xr[10]))+((Xdu+Ydv)*sin(2*Xr[10])-m*sin(Xr[10]))*(Ur[0]+Ur[1]+Ur[2]+Ur[3]))/1.414/m/m;
	
	Ar[2][3] = 1;
	
	Ar[3][1] = -(Yv*sin(Xr[10])*(m-Xdu*cos(Xr[10])))/m/m;
	Ar[3][3] = (Yv*cos(Xr[10])*(m-Xdu*cos(Xr[10])))/m/m;
	Ar[3][10] = (1.414*Xdu*Yv*(Xr[1]*cos(2*Xr[10])+Xr[3]*sin(2*Xr[10]))-1.414*m*Yv*(Xr[1]*cos(Xr[10])+Xr[3]*sin(Xr[10]))+(m*cos(Xr[10])-(Xdu+Ydv)*cos(2*Xr[10]))*(Ur[0]+Ur[1]+Ur[2]+Ur[3]))/1.414/m/m;
	
	Ar[4][5] = 1;
	
	Ar[5][5] = Zw/(m-Zdw);
	
	Ar[6][7] = 1;
	
	Ar[7][7] = Pp/(Ixx-Pdp);
	Ar[7][9] = Xr[11]*(Iyy-Izz)/(Ixx-Pdp);
	Ar[7][11] = Xr[9]*(Iyy-Izz)/(Ixx-Pdp);
	
	Ar[8][9] = 1;
	
	Ar[9][7] = Xr[11]*(Izz-Ixx)/(Iyy-Qdq);
	Ar[9][9] = Qq/(Iyy-Qdq);
	Ar[9][11] = Xr[7]*(Izz-Ixx)/(Iyy-Qdq);
	
	Ar[10][11] = 1;
	
	Ar[11][7] = Xr[9]*(Ixx-Iyy)/(Izz-Rdr);
	Ar[11][9] = Xr[9]*(Ixx-Iyy)/(Izz-Rdr);
	Ar[11][11] = Rr/(Izz-Rdr);
}

void getBr(float* Xr, float* Ur){
	Br[1][0] = (2*m*(cos(Xr[10])-sin(Xr[10]))+(Xdu+Ydv)*(sin(2*Xr[10])-cos(2*Xr[10]))+Xdu-Ydv)/2/1.414/m/m;
	Br[1][1] = (2*m*(cos(Xr[10])+sin(Xr[10]))-(Xdu+Ydv)*(sin(2*Xr[10])+cos(2*Xr[10]))+Xdu-Ydv)/2/1.414/m/m;
	Br[1][2] = (2*m*(cos(Xr[10])-sin(Xr[10]))+(Xdu+Ydv)*(sin(2*Xr[10])-cos(2*Xr[10]))+Xdu-Ydv)/2/1.414/m/m;
	Br[1][3] = (2*m*(cos(Xr[10])+sin(Xr[10]))-(Xdu+Ydv)*(sin(2*Xr[10])+cos(2*Xr[10]))+Xdu-Ydv)/2/1.414/m/m;
	
	Br[3][0] = (2*m*(cos(Xr[10])+sin(Xr[10]))-(Xdu+Ydv)*(sin(2*Xr[10])+cos(2*Xr[10]))-Xdu+Ydv)/2/1.414/m/m;
	Br[3][1] = (-2*m*(cos(Xr[10])-sin(Xr[10]))-(Xdu+Ydv)*(sin(2*Xr[10])-cos(2*Xr[10]))+Xdu-Ydv)/2/1.414/m/m;
	Br[3][2] = (2*m*(cos(Xr[10])+sin(Xr[10]))-(Xdu+Ydv)*(sin(2*Xr[10])+cos(2*Xr[10]))-Xdu+Ydv)/2/1.414/m/m;
	Br[3][3] = (-2*m*(cos(Xr[10])-sin(Xr[10]))-(Xdu+Ydv)*(sin(2*Xr[10])-cos(2*Xr[10]))+Xdu-Ydv)/2/1.414/m/m;
	
	Br[5][4] = 1/(m-Zdw);
	Br[5][5] = 1/(m-Zdw);
	Br[5][6] = 1/(m-Zdw);

	Br[7][0] = lhz/1.414/(Ixx-Pdp);
	Br[7][1] = -lhz/1.414/(Ixx-Pdp);
	Br[7][2] = lhz/1.414/(Ixx-Pdp);
	Br[7][3] = -lhz/1.414/(Ixx-Pdp);
	Br[7][4] = -lvy/(Ixx-Pdp);   //
	Br[7][5] = lvy/(Ixx-Pdp);    //
	
	Br[9][0] = -lhz/1.414/(Iyy-Qdq);
	Br[9][1] = -lhz/1.414/(Iyy-Qdq);
	Br[9][2] = -lhz/1.414/(Iyy-Qdq);
	Br[9][3] = -lhz/1.414/(Iyy-Qdq);
	Br[9][4] = -lvx/1.414/(Iyy-Qdq);
	Br[9][5] = -lvx/1.414/(Iyy-Qdq);
	Br[9][6] = lmx/1.414/(Iyy-Qdq);
	
	Br[11][0] = -1.414*lh/(Izz-Rdr);
	Br[11][1] = 1.414*lh/(Izz-Rdr);
	Br[11][2] = 1.414*lh/(Izz-Rdr);
	Br[11][3] = -1.414*lh/(Izz-Rdr);
	
}

void setXr(void){
	if(PATHPOINT<PATHLENGTH-1&&pow(X[0]-PATH[PATHPOINT][0],2)+pow(X[2]-PATH[PATHPOINT][1],2)<3){
		PATHPOINT++;
	}
	Xr[0] = PATH[PATHPOINT][0];
	Xr[1] = 0;
	Xr[2] = PATH[PATHPOINT][1];
	Xr[3] = 0;
	Xr[4] = 0;
	Xr[5] = 0;
	Xr[6] = 0;
	Xr[7] = 0;
	Xr[8] = 0;
	Xr[9] = 0;
	Xr[10] = getOri(PATHPOINT);
	Xr[11] = 0;
}

float getOri(u16 p){
	double ori = 0;
	if(p>0)
		ori += atan2(PATH[p][1]-PATH[p-1][1],PATH[p][0]-PATH[p-1][0]);
	if(p<PATHPOINT-1){
		ori += atan2(PATH[p+1][1]-PATH[p][1],PATH[p+1][0]-PATH[p][0]);
		ori /= 2;
	}
	while((float)ori>PI) ori -= 2*PI;
	while((float)ori<-PI) ori += 2*PI;
	return (float)ori;
}

void discretization(void){
	u16 i,j;
	for(i=0;i<12;i++){
		for(j=0;j<12;j++){
			Ah[i][j] = T*Ar[i][j]+I[i][j];
		}
		for(j=0;j<7;j++){
			Bh[i][j] = T*Br[i][j];
		}
	}
}
