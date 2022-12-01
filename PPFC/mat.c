#include "mat.h"

void MatInv(float** MatInv, float** Mat, u16 rc)
{
	u16 i, j, k;
	float m, t, tC;
	u16 n;
	MatI((float**)MatInv, rc, 1);
	for(i=0; i<rc; i++){
		m = fabs(*((float*)Mat+rc*i+i));
		n = i;
		for(j=i+1; j<rc; j++){
			if(fabs(*((float*)Mat+rc*j+i))>m){
				m = fabs(*((float*)Mat+rc*j+i));
				n = j;
			}
		}
		if(n!=i){
			for (k=0; k<rc; k++){
				t = *((float*)Mat+rc*i+k);
				*((float*)Mat+rc*i+k) = *((float*)Mat+rc*n+k);
				*((float*)Mat+rc*n+k) = t;
				t = *((float*)MatInv+rc*i+k);
				*((float*)MatInv+rc*i+k) = *((float*)MatInv+rc*n+k);
				*((float*)MatInv+rc*n+k) = t;
			}
		}
		if(*((float*)Mat+rc*i+i)==(float)0) return;
		t = *((float*)Mat+rc*i+i);
		for(j=0; j<rc; j++){
			*((float*)Mat+rc*i+j) /= t;
			*((float*)MatInv+rc*i+j) /= t;
		}
		for(j=0; j<rc; j++){
			if(i!=j){
				t = *((float*)Mat+rc*j+i);
				for(k=0; k<rc; k++){
					*((float*)Mat+rc*j+k) = *((float*)Mat+rc*j+k) - *((float*)Mat+rc*i+k) * t;
					*((float*)MatInv+rc*j+k) = *((float*)MatInv+rc*j+k) - *((float*)MatInv+rc*i+k) * t;
				}
			}
		}
	}
}

void MatCopy(float** MatA, float** MatB, u16 R, u16 C){
	u16 r, c;
	for(r=0; r<R; r++)
		for(c=0; c<C; c++)
			*((float*)MatA+C*r+c) = *((float*)MatB+C*r+c);
}

void MatI(float** Mat, u16 rc, float k){
	u16 r=0, c=0;
	for(r=0; r<rc; r++)
		for(c=0; c<rc; c++)
			*((float*)Mat+rc*r+c) = r==c?k:0;
}

void MatMul(float** Mat, float** MatA, float** MatB, u16 I, u16 J, u16 K, u16 SA, u16 SB, u16 SC){
	u16 i, j, k, a, b;
	if(SC==0)
		for(i=0;i<I;i++)
			for(j=0;j<J;j++)
				*((float*)Mat+J*i+j) = 0;
	for(i=0;i<I;i++)
		for(j=0;j<J;j++){
			for(k=0;k<K;k++){
				a = SA==1?I*k+i:K*i+k;
				b = SB==1?K*j+k:J*k+j;
				*((float*)Mat+J*i+j) += *((float*)MatA+a)*(*((float*)MatB+b));
			}
		}
}

void MatAdd(float** Mat, float** MatA, float** MatB, u16 R, u16 C){
	u16 r, c;
	for(r=0; r<R; r++)
		for(c=0; c<C; c++)
			*((float*)Mat+C*r+c) = *((float*)MatA+C*r+c)+*((float*)MatB+C*r+c);
}
