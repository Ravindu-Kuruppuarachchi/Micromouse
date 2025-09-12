#include "wallFollow.h"
#include "stdio.h"
float prevError = 0;

int calculateCorrection(uint16_t d1,uint16_t d2,float Kp,float Kd){
	float Error = d1 - d2;
	float derivative = Error-prevError;

	prevError = Error;

	int Correction = ((Kp*Error)+Kd*derivative);
	return Correction;
}


