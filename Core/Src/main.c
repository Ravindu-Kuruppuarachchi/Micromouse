/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "VL53L0X.h"
#include "stdio.h"
#include "motors.h"
#include "wallFollow.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void scanI2CBus(void) {
    char buffer[16];
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("I2C Scan:", &Font_11x18, 1);

    for (uint8_t address = 1; address < 128; address++) {
        // Perform an I2C write operation to check if the device responds
        if (HAL_I2C_IsDeviceReady(&hi2c1, (address << 1), 1, 10) == HAL_OK) {
            snprintf(buffer, sizeof(buffer), "Found: 0x%02X", address);
            SSD1306_GotoXY(0, 30);
            SSD1306_Puts(buffer, &Font_11x18, 1);
            SSD1306_UpdateScreen();
            HAL_Delay(1000);
            //SD1306_Clear();// Delay to display each address found
        }
    }
    HAL_Delay(2000);  // Delay at the end of the scan
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);  // Enable Sensor 1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);  // Disable Sensor 2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  HAL_Delay(50);
  if (VL53L0X_Init(0x29) == HAL_OK)
  {
	  SSD1306_GotoXY(0, 30);
  	  SSD1306_Puts("Success", &Font_11x18, 1);
  	  HAL_Delay(2000);
  	  SSD1306_UpdateScreen();
    }
  else
  {
  	  SSD1306_GotoXY(0, 30);
  	  SSD1306_Puts("Faild8", &Font_11x18, 1);
  	  HAL_Delay(2000);
  	  SSD1306_UpdateScreen();
  }

  if (SetAddress(0x31) == HAL_OK)
  {
	  SSD1306_GotoXY(0, 30);
  	  SSD1306_Puts("Success", &Font_11x18, 1);
  	  HAL_Delay(2000);
  	  SSD1306_UpdateScreen();
   }
   else
   {
  	  SSD1306_GotoXY(0, 30);
  	  SSD1306_Puts("Faild8", &Font_11x18, 1);
  	  HAL_Delay(2000);
  	  SSD1306_UpdateScreen();
   }

    HAL_Delay(10);

    SSD1306_GotoXY(0, 30);
    SSD1306_Puts("Initialized1", &Font_11x18, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(1000);
    SSD1306_Clear();

    //scanI2CBus();
    SSD1306_Clear();

      // Enable Sensor 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);  // Disable Sensor 2
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
    HAL_Delay(50);
    if (VL53L0X_Init(0x29) == HAL_OK)
    {
    	SSD1306_GotoXY(0, 30);
      	SSD1306_Puts("Success", &Font_11x18, 1);
      	HAL_Delay(2000);
      	SSD1306_UpdateScreen();
     }
    else
     {
    	SSD1306_GotoXY(0, 30);
      	SSD1306_Puts("Faild8", &Font_11x18, 1);
      	HAL_Delay(2000);
      	SSD1306_UpdateScreen();
      }

    if (SetAddress(0x32) == HAL_OK)
      {
    	SSD1306_GotoXY(0, 30);
      	SSD1306_Puts("Success", &Font_11x18, 1);
      	HAL_Delay(2000);
      	SSD1306_UpdateScreen();
       }
    else
       {
      	SSD1306_GotoXY(0, 30);
      	SSD1306_Puts("Faild8", &Font_11x18, 1);
      	HAL_Delay(2000);
      	SSD1306_UpdateScreen();
        }

    HAL_Delay(10);

    SSD1306_GotoXY(0, 30);
    SSD1306_Puts("Initialized1", &Font_11x18, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(1000);
    SSD1306_Clear();

    //scanI2CBus();
    SSD1306_Clear();


    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
    HAL_Delay(50);
    if (VL53L0X_Init(0x29) == HAL_OK)
    {
    	SSD1306_GotoXY(0, 30);
        SSD1306_Puts("Success", &Font_11x18, 1);
        HAL_Delay(2000);
        SSD1306_UpdateScreen();
     }
    else
     {
    	SSD1306_GotoXY(0, 30);
        SSD1306_Puts("Faild8", &Font_11x18, 1);
        HAL_Delay(2000);
        SSD1306_UpdateScreen();
      }

    if (SetAddress(0x33) == HAL_OK)
       {
    	SSD1306_GotoXY(0, 30);
        SSD1306_Puts("Success", &Font_11x18, 1);
        HAL_Delay(2000);
        SSD1306_UpdateScreen();
        }
    else
        {
    	SSD1306_GotoXY(0, 30);
        SSD1306_Puts("Faild8", &Font_11x18, 1);
        HAL_Delay(2000);
        SSD1306_UpdateScreen();
         }

    HAL_Delay(10);

    SSD1306_GotoXY(0, 30);
    SSD1306_Puts("Initialized1", &Font_11x18, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(1000);
    SSD1306_Clear();

    scanI2CBus();
    SSD1306_Clear();

//    uint16_t distance1;
//    uint16_t distance2;
//    uint16_t distance3;
    char buffer1[16];
    char buffer2[16];
    char buffer3[16];


  int counterValueLeft=0;
  int counterValueRight=0;

  int precounterValueLeft=0;
  int precounterValueRight=0;
//  float integral = 0;
//  float prevError = 0;
//  int leftPWM = 150;
//  int rightPWM = 150;
//  int targetSpeed = 130;


//  float Kpencoder = 0.08;
//  float Kiencoder = 0;
//  float Kdencoder = 0.1;

  char snumLeft[5];
  char snumRight[5];

//    int constrain(int value, int min, int max) {
//        if (value < min) return min;
//        if (value > max) return max;
//        return value;
//    }


//  SSD1306_GotoXY (0,0);
//  SSD1306_Puts ("Ravindu", &Font_11x18, 1);
//  SSD1306_GotoXY (0, 30);
//  SSD1306_Puts ("Rashmika", &Font_11x18, 1);
//  SSD1306_UpdateScreen();
//  HAL_Delay (1000);
//
//  SSD1306_ScrollRight(0,7);
//  HAL_Delay(3000);
//  SSD1306_ScrollLeft(0,7);
//  HAL_Delay(3000);
//  SSD1306_Stopscroll();
  SSD1306_Clear();

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

  TIM2->CNT=0;
  TIM3->CNT=0;

  char rightStr[10], leftStr[10], errorStr[15];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  distance1 = readDistance(0x31);
//	  distance2 = readDistance(0x32);
//	  distance3 = readDistance(0x33);

	  //cellBreak();

//	  SSD1306_GotoXY(0, 0); // Set cursor position for distance1
//	  SSD1306_Puts("D1: ", &Font_11x18, 1); // Prefix for clarity
//	  if (distance1 == 0xFFFF) {
//	      SSD1306_Puts("Err", &Font_11x18, 1); // Display "Err" for invalid data
//	  } else {
//	      snprintf(buffer1, sizeof(buffer1), "%4d", distance1); // Format with fixed width
//	      SSD1306_Puts(buffer1, &Font_11x18, 1);
//	  }
//
//	  SSD1306_GotoXY(0, 20); // Set cursor position for distance2
//	  SSD1306_Puts("D2: ", &Font_11x18, 1); // Prefix for clarity
//	  if (distance2 == 0xFFFF) {
//	      SSD1306_Puts("Err", &Font_11x18, 1); // Display "Err" for invalid data
//	  } else {
//	      snprintf(buffer2, sizeof(buffer2), "%4d", distance2); // Format with fixed width
//	      SSD1306_Puts(buffer2, &Font_11x18, 1);
//	  }
//
//
//	  SSD1306_GotoXY(0, 40); // Set cursor position for distance3
//	  SSD1306_Puts("D3: ", &Font_11x18, 1); // Prefix for clarity
//	  if (distance3 == 0xFFFF) {
//	      SSD1306_Puts("Err", &Font_11x18, 1); // Display "Err" for invalid data
//	  } else {
//	      snprintf(buffer3, sizeof(buffer3), "%4d", distance3); // Format with fixed width
//	      SSD1306_Puts(buffer3, &Font_11x18, 1);
//	  }
//
//	  SSD1306_UpdateScreen();
	  //HAL_Delay(200); // Delay for readability

//
//	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
//	  	  HAL_Delay(50);
//	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
////	  	  HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_ALL);
////	  	  HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_ALL);
//
//	  	  counterValueRight = TIM3->CNT;
//	  	  counterValueLeft = TIM2->CNT;
//

	  	  //int delta_left = counterValueLeft-precounterValueLeft;
	  	  //int delta_right = counterValueRight-precounterValueRight;

	  	  //float Error = distance1 - distance2;
	  	  //float derivative = Error-prevError;
//	  	if (abs(integral) > 50) integral = 50;  // Prevents excessive integral buildup
//	  	integral += Error / 10.0;
//
//	  	  float derivative = (Error-prevError)/10.0;
	  	  //prevError = Error;

//	  	if (counterValueRight > 60000 || counterValueLeft > 60000) {
//	  		TIM3->CNT = Error + 30000;
//	  		TIM2->CNT = 30000;
//	  	}


//	  	  int Correction = calculateCorrection(distance1,distance2,Kpencoder,Kdencoder);

//	  		leftPWM = targetSpeed+Correction;
//	  		rightPWM = targetSpeed-Correction;

	  	  // Adjust both motors symmetrically
  // Apply correction to right motor as well

//	  	  leftPWM = constrain(leftPWM, 0, 200);
//	  	  rightPWM = constrain(rightPWM, 0, 200);
//
//	  	  foward();
	  solver();

//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);  // Enable motor LEFT
//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
//	  		  	  TIM1->CCR1 = leftPWM;
//	  		  	  TIM1->CCR2 = rightPWM;// 50% duty cycle for TIM_CHANNEL_1
//	  		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	  		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	  //	  itoa(leftPWM, snumLeft, 10);
	  //	  itoa(rightPWM, snumRight, 10);
	  //	  SSD1306_GotoXY (37, 10);
	  //	  SSD1306_Puts (snumLeft, &Font_11x18, 1);
	  //	  SSD1306_GotoXY (37, 40);
	  //	  SSD1306_Puts (snumRight, &Font_11x18, 1);
	  //
	  //	  SSD1306_UpdateScreen();
//
//	  	  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//	  	  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
//
////
//	      sprintf(rightStr, "E: %d", distance1);
//	      sprintf(leftStr, "D: %d",distance2 );
//	      //sprintf(errorStr, "C: %d", Correction);
//
//	      // Display on OLED
//	      SSD1306_Clear();
//	      SSD1306_GotoXY(10, 5);
//	      SSD1306_Puts(leftStr, &Font_11x18, 1); // Left encoder
//	      SSD1306_GotoXY(10, 25);
//	      SSD1306_Puts(rightStr, &Font_11x18, 1); // Right encoder
//	      SSD1306_GotoXY(10, 45);
//	      SSD1306_Puts(errorStr, &Font_11x18, 1);
//	      SSD1306_UpdateScreen(); // Refresh OLED

//	      if (counterValueRight > 60000 || counterValueLeft > 60000) {
//	      	  		TIM3->CNT = (distance1 - distance2) + 30000;
//	      	  		TIM2->CNT = 30000;
//	      	  	}

//	      if (counterValueRight > 1800 || counterValueLeft > 1800) {
//	    	  	  	  	  	stop();
//	      	      	  		TIM3->CNT = 0;
//	      	      	  		TIM2->CNT = 0;
//
//	      	      	  	SSD1306_Clear();
//	      	      	  			SSD1306_GotoXY(0, 30);
//	      	      	  			    SSD1306_Puts("Done", &Font_11x18, 1);
//	      	      	  			    SSD1306_UpdateScreen();
//	      	      	  			    HAL_Delay(1000);
//	      	      	  			    SSD1306_Clear();
//	      	      	  			    HAL_Delay(3000);
//	      }

//	      if(distance1>200 || distance2>200){
//	    	  stop();
//	    	  HAL_Delay(2000);
//	    	  Turnright();
//	    	  HAL_Delay(2000);
//	      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
