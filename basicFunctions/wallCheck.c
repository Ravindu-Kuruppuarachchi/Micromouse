#include "stdio.h"
#include "wallCheck.h"
#include "VL53L0X.h"

uint16_t distance1;
uint16_t distance2;
uint16_t distance3;

int wallFront(){
	distance3 = readDistance(0x33);
	if(distance3<300){
		return 1;
	}
	else{
		return 0;
	}
}

int wallLeft(){
	distance2 = readDistance(0x32);
	if(distance2<300){
		return 1;
	}
	else{
		return 0;
	}
}

int wallRight(){
	distance3 = readDistance(0x31);
	if(distance1<300){
		return 1;
	}
	else{
		return 0;
	}
}
