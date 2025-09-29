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
#include "bno055_stm32.h"
#include "VL53L0X.h"
#include "pca9685.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t rx_data_3;
uint8_t rx_data_6;
uint8_t Linear_flag = 0;
uint8_t Gyro_flag = 0;
uint8_t Left_flag = 0;
uint8_t Right_flag = 0;
uint8_t Forward_flag = 0;
uint8_t Backward_flag = 0;
uint8_t Servo_open = 0;
uint8_t Servo_close = 0;
uint8_t i;

uint8_t serialBuf[100];
uint8_t IR_flag = 0;
float heading = 0.0f;
uint8_t heading_initialized = 0;
float delta_3 = 0.0f;
float delta_4 = 0.0f;

float raw = 0.0f;
float raw_3 = 0.0f;
float raw_4 = 0.0f;
float Tim3_raw = 0.0f;
float Tim4_raw = 0.0f;
uint8_t delta_count = 0;
uint8_t correcting_direction = 0;
uint8_t tim4_initialized = 0;
uint8_t stop_sent = 0;
uint8_t  Lifting_check = 0;
float init_heading = 0.0f;      // s에서 저장한 초기 heading
uint8_t return_mode = 0;        // f 동작으로 초기값 복귀 모드
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart6, &rx_data_6, 1);
  HAL_UART_Receive_IT(&huart3, &rx_data_3, 1);

  char msgBuffer[52];
  for (uint8_t i = 0; i < 52; i++) {
	  msgBuffer[i] = ' ';
  }

  PCA9685_Init(50); // 50Hz
  bno055_assignI2C(&hi2c2);
  bno055_setup();
  bno055_setOperationModeNDOF();

  sprintf(msgBuffer, "BNO start \r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*) msgBuffer, strlen(msgBuffer), 50);

  HAL_Delay(20);
  PCA9685_Init(50); // 50Hz

  PCA9685_SetServoAngle(0, 123);
  PCA9685_SetServoAngle(1, 110);	//오른쪽 위
  PCA9685_SetServoAngle(2, 115);    //왼쪽 아래
  PCA9685_SetServoAngle(3, 117);
  HAL_Delay(500);

	sprintf(msgBuffer, "Servo start \r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*) msgBuffer, strlen(msgBuffer), 50);




  //Initialise the VL53L0X
  statInfo_t_VL53L0X distanceStr;
  initVL53L0X(1, &hi2c3);
  startContinuous(20);

  sprintf(msgBuffer, "VL53L0X start \r\n");
  	HAL_UART_Transmit(&huart3, (uint8_t*) msgBuffer, strlen(msgBuffer), 50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if (IR_flag == 1) {
	      uint8_t distance = readRangeSingleMillimeters(&distanceStr);
	      sprintf(msgBuffer, "Distance: %d\r\n", distance);
	      HAL_UART_Transmit(&huart3, (uint8_t*) msgBuffer, strlen(msgBuffer), 50);
	      HAL_Delay(20);

	      if (distance < 180) {
	          IR_flag = 0;
	          sprintf((char *)serialBuf, "9");
	          HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
	      }
	  }

	  else if (Servo_open == 1) {


	      for (i = 0; i <= 97; i++) {

 	          PCA9685_SetServoAngle(0, 123 - i);
 	          PCA9685_SetServoAngle(1, 110 + i);
 	          PCA9685_SetServoAngle(2, 115 + i);
 	          PCA9685_SetServoAngle(3, 117 - i);
	          HAL_Delay(10);
	      }

	      Servo_open = 0;
	      Lifting_check = 1;



	  }

	  else if (Servo_close == 1) {

	      for (i = 0; i <= 97; i++) {

	          PCA9685_SetServoAngle(0, 26 + i);
			  PCA9685_SetServoAngle(1, 207 -  i);
			  PCA9685_SetServoAngle(2, 212 - i);
			  PCA9685_SetServoAngle(3, 20 + i);
			  HAL_Delay(10);
	      }

	      Servo_close = 0;
	      Lifting_check = 0;



	  }
//
//	  else if (Linear_flag == 1) {
//	      sprintf((char *)serialBuf, "3");
//	      HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
//
//
//
//	      Linear_flag = 0;
//	  }


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  hi2c1.Init.ClockSpeed = 100000;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1599;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM3)	//회전 각도 값 받아오기
    {

        bno055_vector_t v = bno055_getVectorEuler();
        raw_3 = v.x - heading;

        // -180 ~ 180 범위로 보정
        if (raw_3 > 180.0f)
            raw_3 -= 360.0f;
        else if (raw_3 < -180.0f)
            raw_3 += 360.0f;

        delta_3 = fabsf(raw_3);

//        sprintf((char *)serialBuf, "heading = %.2f, v.x = %.2f, delta = %.2f\r\n", heading, v.x, delta_3);
//        HAL_UART_Transmit(&huart3, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);

        // 멈춤 조건: 방향 상관없이 90도 이상 회전 시
        if ((delta_3 >= 89.1f &&  Lifting_check == 0) || (delta_3 >= 88.5f &&  Lifting_check == 1)) {
            delta_count++;
        }

        if (delta_count >= 2) {
        	if(stop_sent == 0)
        	{
				sprintf((char *)serialBuf, "4");	//동작 정지 신
				HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
        	}

            HAL_TIM_Base_Stop_IT(&htim3);
             Left_flag = 0;
             Right_flag = 0;

             stop_sent = 1;


            delta_count = 0;

//            //보정변수 초기화
//            correcting_direction = 0;
//			raw_4 = 0.0f;
//			Forward_flag = 0;
//			Backward_flag = 0;
        }
    }

    else if (htim->Instance == TIM4) {	//주행중 각도 보정
        bno055_vector_t v = bno055_getVectorEuler();
         raw_4 = v.x - heading;

         if (raw_4 > 180.0f)
             raw_4 -= 360.0f;
         else if (raw_4 < -180.0f)
             raw_4 += 360.0f;

         delta_4 = raw_4;
         if (return_mode == 0) {
//        sprintf((char *)serialBuf, "raw = %.2f\r\n", delta_4);
//        HAL_UART_Transmit(&huart3, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);

			if (correcting_direction == 0) {
				if (delta_4 >= 1.2f) { // 오른쪽으로 휨
					sprintf((char *)serialBuf, "5"); // 왼쪽 회전 시작
					HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
					correcting_direction = 1;
				} else if (delta_4 <= -1.2f) { // 왼쪽으로 휨
					sprintf((char *)serialBuf, "6"); // 오른쪽 회전 시작
					HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
					correcting_direction = 2;
				}
			} else if (correcting_direction == 1) { // 오른쪽 회전 중
				if (delta_4 <= 0.3f) {
					if (Forward_flag == 1) {
						sprintf((char *)serialBuf, "7"); // 다음 명령(전진)
						HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
						correcting_direction = 0; // 상태 초기화
					} else if (Backward_flag == 1) {
						sprintf((char *)serialBuf, "8"); // 다음 명령(후진)
						HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
						correcting_direction = 0; // 상태 초기화
					}
				}
			} else if (correcting_direction == 2) { // 왼쪽 회전 중
				if (delta_4 >= -0.3f) {
					if (Forward_flag == 1) {
						sprintf((char *)serialBuf, "7"); // 다음 명령(전진)
						HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
						correcting_direction = 0; // 상태 초기화
					} else if (Backward_flag == 1) {
						sprintf((char *)serialBuf, "8"); // 다음 명령(후진)
						HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
						correcting_direction = 0; // 상태 초기화
					}
				}
			}
         }
	 else if (return_mode == 1) {
			 /* ===== f: 초기 heading으로 복귀 ===== */
			 float delta = raw_4;

			 if (fabsf(delta) < 0.5f) {
				 // 오차 1도 이내 → 복귀 완료
				 sprintf((char *)serialBuf, "4"); // 다음 명령(후진)
				 HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);

				 HAL_TIM_Base_Stop_IT(&htim4);
				 return_mode = 0;  // 모드 종료
			 } else {
				 if (delta > 0) {
					 sprintf((char*)serialBuf, "5"); // 왼쪽으로 회전
					 HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);
				 } else {
					 sprintf((char*)serialBuf, "6"); // 오른쪽으로 회전
					 HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);
				 }
			 }
		 }
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        if (rx_data_6 == '1') {
            IR_flag = 1;

        }

        else if (rx_data_6 == '2') {
            Linear_flag = 1;
        }

        else if (rx_data_6 == '3') { // 레프트턴
        	HAL_TIM_Base_Stop_IT(&htim4);
        	stop_sent = 0;
            Left_flag = 1;
			Right_flag = 0;
			heading = bno055_getVectorEuler().x;
			HAL_TIM_Base_Start_IT(&htim3);
        }

        else if (rx_data_6 == '4') { // 라이트턴
        	HAL_TIM_Base_Stop_IT(&htim4);
        	stop_sent = 0;
        	Left_flag = 0;
            Right_flag = 1;
            heading = bno055_getVectorEuler().x;
            HAL_TIM_Base_Start_IT(&htim3);
        }

        else if (rx_data_6 == '5') {
            Forward_flag = 1;
            Backward_flag = 0;
			heading = bno055_getVectorEuler().x;
            HAL_TIM_Base_Start_IT(&htim4);
        }

        else if (rx_data_6 == '6') {
            Forward_flag = 0;
            Backward_flag = 1;
			heading = bno055_getVectorEuler().x;
            HAL_TIM_Base_Start_IT(&htim4);
        }

        else if (rx_data_6 == '7') {
        	HAL_TIM_Base_Stop_IT(&htim4);
        	HAL_TIM_Base_Stop_IT(&htim3);

		// 회전 변수 초기화
			delta_count = 0;
			raw_3 = 0.0f;
			Left_flag = 0;
			Right_flag = 0;

			// 보정 변수 초기화
			correcting_direction = 0;
			raw_4 = 0.0f;
			Forward_flag = 0;
			Backward_flag = 0;

			// 공통 delta도 초기화
			delta_3 = 0.0f;
        }

        else if (rx_data_6 == '8') {
        	Servo_close = 1;
        }

        else if (rx_data_6 == '9') {
        	Servo_open = 1;
        }
        else if (rx_data_6 == 's') {
            // 시뮬 시작 → 초기 heading 저장
            bno055_vector_t v = bno055_getVectorEuler();
            init_heading = v.x;
            return_mode = 0;  // 보정 모드 아님

        } else if (rx_data_6 == 'f') {
            // 시뮬 종료 → 초기값 복귀 모드 진입
            return_mode = 1;
            heading = init_heading;   // TIM4에서 비교 기준으로 사용
            HAL_TIM_Base_Start_IT(&htim4);  // 보정 타이머4 시작
        }
        else if (rx_data_6 == 'c') {
		   // 시뮬 종료 → 초기값 복귀 모드 진입
		   return_mode = 1;
		   heading = init_heading + 185.0f;   // TIM4에서 비교 기준으로 사용
		   HAL_TIM_Base_Start_IT(&htim4);  // 보정 타이머4 시작
	   }

        // 다음 수신 대기
        HAL_UART_Receive_IT(&huart6, &rx_data_6, 1);
    }

    else if (huart->Instance == USART3) {
        if (rx_data_3 == '1') {
            IR_flag = 1;
            sprintf((char *)serialBuf, "4");
            HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
        }

        else if (rx_data_3 == '2') {
            sprintf((char *)serialBuf, "2");
            HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
        }

        else if (rx_data_3 == '3') {

            HAL_TIM_Base_Start_IT(&htim3);
        }

        else if (rx_data_3 == '4') {
            sprintf((char *)serialBuf, "4");
            HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
        }

        else if (rx_data_3 == '5') {
        	Servo_open = 1;
        }

        else if (rx_data_3 == '6') {
        	Servo_close = 1;
        }

        else if (rx_data_3 == '7') {
            HAL_TIM_Base_Start_IT(&htim4);
        }

        else if (rx_data_3 == '8') {
            Linear_flag = 1;
        }

        // 다음 수신 대기
        HAL_UART_Receive_IT(&huart3, &rx_data_3, 1);
    }
}

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
#ifdef USE_FULL_ASSERT
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
