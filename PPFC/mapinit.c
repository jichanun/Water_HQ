#include "mapinit.h"
#include "planner.h"

void Map_Init(void){
	u16 i,j;
	for(i=0; i<MR; i++){
		for(j=0; j<MC; j++){
			MAP[i][j] = i==0||j==0||i==MR-1||j==MC-1?1:0;
		}
	}
}
