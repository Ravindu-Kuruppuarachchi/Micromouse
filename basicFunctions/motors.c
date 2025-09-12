#include "motors.h"
#include "stdio.h"
#include "ssd1306.h"
#include "VL53L0X.h"
#include "wallFollow.h"
//#include "solver.h"
//#include "constants.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

const int TARGET_COUNT = 320;
float error = 0, previousError = 0;
float integral = 0, derivative = 0;
float pidOutput = 0;

float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

int rightEn = 0;
int leftEn = 0;

char snum[5];

float diameter = 3.2; //mm
float circumferance = 10.048;
int t_per = 10.048/360;
int avg = 0;

int Correction;
float Kpencoder = 0.08;
float Kiencoder = 0;
float Kdencoder = 0.1;

int leftPWM;
int rightPWM;
int targetSpeed;
extern uint16_t distance1;
extern uint16_t distance2;
extern uint16_t distance3;


char leftEncoder[10];
char rightEncoder[10];
char strError[10];


int countValLeftCorr = 0;
int countValRightCorr = 0;

int errorT;
float integralT;
float derivativeT;
int TargetCount;
float prevErrorT;
float pidOutputT;

float KpTurn = 0.95;
float KiTurn = 0.00;
float KdTurn = 0;

int counterValueRight;
int counterValueLeft;

//void foward() {
//    if (!checkWall(currentHeading, currentXY)) {
//    	counterValueRight = 0;
//    	counterValueLeft = 0;
//    	targetSpeed = 130;
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
//        HAL_Delay(50);
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
//
//
//    	counterValueRight = TIM3->CNT;
//    	counterValueLeft = TIM2->CNT;
//
//    	distance1 = readDistance(0x31);
//    	distance2 = readDistance(0x32);
//
//    	Correction = calculateCorrection(distance1,distance2,Kpencoder,Kdencoder);
//
//    	leftPWM = targetSpeed+Correction;
//    	rightPWM = targetSpeed-Correction;
//
//    	leftPWM = constrain(leftPWM, 0, 200);
//    	rightPWM = constrain(rightPWM, 0, 200);
//
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
//    		TIM1->CCR1 = leftPWM;
//    		TIM1->CCR2 = rightPWM;
//    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//
//    	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//    	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
//
//    	 if (counterValueRight > 1275 || counterValueLeft > 1275) {
//    		    	  	  	  	  	stop();
//    		      	      	  		TIM3->CNT = 0;
//    		      	      	  		TIM2->CNT = 0;
//
//    		      	      	  	SSD1306_Clear();
//    		      	      	  			SSD1306_GotoXY(0, 30);
//    		      	      	  			    SSD1306_Puts("Done", &Font_11x18, 1);
//    		      	      	  			    SSD1306_UpdateScreen();
//    		      	      	  			    HAL_Delay(1000);
//    		      	      	  			    SSD1306_Clear();
//    		      	      	  			    HAL_Delay(3000);
//    		 }  // Replace with your function that moves straight
//    }
//}



void foward(){
	counterValueRight = 0;
	counterValueLeft = 0;
	targetSpeed = 130;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
    HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);


	counterValueRight = TIM3->CNT;
	counterValueLeft = TIM2->CNT;

	distance1 = readDistance(0x31);
	distance2 = readDistance(0x32);

	Correction = calculateCorrection(distance1,distance2,Kpencoder,Kdencoder);

	leftPWM = targetSpeed+Correction;
	rightPWM = targetSpeed-Correction;

	leftPWM = constrain(leftPWM, 0, 200);
	rightPWM = constrain(rightPWM, 0, 200);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
		TIM1->CCR1 = leftPWM;
		TIM1->CCR2 = rightPWM;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

	 if (counterValueRight > 1275 || counterValueLeft > 1275) {
		    	  	  	  	  	stop();
		      	      	  		TIM3->CNT = 0;
		      	      	  		TIM2->CNT = 0;

		      	      	  	SSD1306_Clear();
		      	      	  			SSD1306_GotoXY(0, 30);
		      	      	  			    SSD1306_Puts("Done", &Font_11x18, 1);
		      	      	  			    SSD1306_UpdateScreen();
		      	      	  			    HAL_Delay(1000);
		      	      	  			    SSD1306_Clear();
		      	      	  			    HAL_Delay(3000);
		 }
}

void stop(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
}

void Turnright(){
	TIM2->CNT=0;
	TIM3->CNT=0;
	errorT = 0;
	integralT = 0;
	derivativeT = 0;
	prevErrorT = 0;
	pidOutputT = 0;
	TargetCount = 315;
	targetSpeed = 150;

	while(1>0){
		  counterValueRight = TIM3->CNT;
		  counterValueLeft = TIM2->CNT;

		  errorT = TargetCount - (counterValueRight);
		  integralT += errorT;
		  derivativeT = errorT - prevErrorT;

		  pidOutputT = (KpTurn * errorT + KiTurn * integralT + KdTurn * derivativeT)/2;
		  targetSpeed = constrain(abs(pidOutputT), 0, 150);

		  sprintf(leftEncoder, "%d", countValLeftCorr);
		  sprintf(rightEncoder, "%d", counterValueRight);
		  sprintf(strError, "%d", errorT);

				  SSD1306_Clear();
				  SSD1306_GotoXY (0, 0);
				  SSD1306_Puts (leftEncoder, &Font_11x18, 1);
				  SSD1306_GotoXY (0, 20);
				  SSD1306_Puts (rightEncoder, &Font_11x18, 1);
				  SSD1306_GotoXY (0, 40);
				  SSD1306_Puts (strError, &Font_11x18, 1);
				  SSD1306_UpdateScreen();

		  if (errorT > 0){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);  // Enable motor LEFT
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
				  TIM1->CCR1 = targetSpeed;
				  TIM1->CCR2 = targetSpeed;// 50% duty cycle for TIM_CHANNEL_1
			  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  }

		  else{
			  break;
		  }
	  }

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);  // Enable motor LEFT
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	  HAL_Delay(1000);
}

void Turnleft(){
	TIM2->CNT=0;
	TIM3->CNT=0;
	errorT = 0;
	integralT = 0;
	derivativeT = 0;
	prevErrorT = 0;
	pidOutputT = 0;
	TargetCount = 320;
	targetSpeed = 150;

	while(1>0){
		  counterValueRight = TIM3->CNT;
		  counterValueLeft = TIM2->CNT;

		  errorT = TargetCount - (counterValueLeft);
		  integralT += errorT;
		  derivativeT = errorT - prevErrorT;

		  pidOutputT = (KpTurn * errorT + KiTurn * integralT + KdTurn * derivativeT)/2;
		  targetSpeed = constrain(abs(pidOutputT), 0, 150);

		  sprintf(leftEncoder, "%d", counterValueLeft);
		  sprintf(rightEncoder, "%d",countValRightCorr);
		  sprintf(strError, "%d", errorT);

				  SSD1306_Clear();
				  SSD1306_GotoXY (0, 0);
				  SSD1306_Puts (leftEncoder, &Font_11x18, 1);
				  SSD1306_GotoXY (0, 20);
				  SSD1306_Puts (rightEncoder, &Font_11x18, 1);
				  SSD1306_GotoXY (0, 40);
				  SSD1306_Puts (strError, &Font_11x18, 1);
				  SSD1306_UpdateScreen();

		  if (errorT > 0){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);  // Enable motor LEFT
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
				  TIM1->CCR1 = targetSpeed;
				  TIM1->CCR2 = targetSpeed;// 50% duty cycle for TIM_CHANNEL_1
			  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  }

		  else{
			  break;
		  }
	  }

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);  // Enable motor LEFT
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	  HAL_Delay(1000);
}

int constrain(int value, int min, int max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
}


void cellBreak(){
	rightEn = TIM3->CNT;
	leftEn = TIM2->CNT;
	avg = (rightEn+leftEn)/2;
	if(avg > 600/t_per){
		TIM2->CNT=0;
		TIM3->CNT=0;
		SSD1306_Clear();
		SSD1306_GotoXY(0, 30);
		    SSD1306_Puts("Done", &Font_11x18, 1);
		    SSD1306_UpdateScreen();
		    HAL_Delay(1000);
		    SSD1306_Clear();
		    HAL_Delay(3000);
		    //scanI2CBus();
		    //SSD1306_Clear();

	}
}
