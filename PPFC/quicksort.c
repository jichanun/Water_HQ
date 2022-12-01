#include "quicksort.h"
#include "planner.h"

void quicksort(u16 len){
	u16 step;
	for(step=0; step<=len; step++) DISTQUEUE[step] = step;
	sort(0, len);
	return;
}

void sort(u16 left, u16 right){
	double t;
	u16 l = left, r = right;
	if(left>=right) return;
	t = DIST[right];
	while(l<r){
		while(l < r && DIST[l] <= t) l++;
		while(l < r && DIST[r] >= t) r--;
		if(DIST[l]>DIST[r])
			swap(l, r);
	}
	swap(l,right);
	if(r>left)
		sort(left, r-1);
	if(r<right)
		sort(r+1,right);
}

void swap(u16 a, u16 b){
	double tv = DIST[a];
	u16 tl = DISTQUEUE[a];
	DIST[a] = DIST[b];
	DIST[b] = tv;
	DISTQUEUE[a] = DISTQUEUE[b];
	DISTQUEUE[b] = tl;
}
