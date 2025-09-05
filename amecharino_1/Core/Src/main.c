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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "string.h"
#include "stdio.h"
#include "VL53L0X.h"
#include "pca9685.h"
#include "vl6180x.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE  1
#define FALSE 0
#define M_PI  3.14159265358979323846

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t  serialBuf[100];
char     rx_buffer[32];
uint8_t  rx_index = 0;
uint8_t  rx_data_3;
uint8_t  rx_data_6;
uint8_t  Lifting_check = 0;
uint8_t  servo_close   = 0;
uint8_t  servo_open    = 0;
uint8_t  forward_flag  = 0;
uint8_t  backward_flag = 0;
uint8_t  right_turn_flag = 0;
uint8_t  left_turn_flag  = 0;
uint8_t  distance;
uint8_t  distancestr;
uint8_t  start = 0;
float    vx_value = 0.0f;

volatile uint8_t Linear_flag      = 0;
volatile uint8_t Linear_num       = 0;
volatile uint8_t Linear_back_flag = 0;
volatile uint8_t Linear_back_num  = 0;

float vx0  = 0.25f;
float kv   = 1.2f;
float vmin = 0.08f;
float Ky   = 0.8f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // MPU6050_Initialization();
  Motor_Init();

  // 아르코 마커러 부터 인덱스 값 받아오는거임
  HAL_UART_Receive_IT(&huart3, &rx_data_3, 1);
  HAL_UART_Receive_IT(&huart6, &rx_data_6, 1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);


  PCA9685_Init(50); // 50Hz
	PCA9685_SetServoAngle(0, 123); // 줄어들기
	PCA9685_SetServoAngle(1, 125); // 늘어나기
	PCA9685_SetServoAngle(2, 110); // 늘어나기
	PCA9685_SetServoAngle(3, 115); // 줄어들기

  VL6180X_Init();

  HAL_Delay(300);

  char msgBuffer[52];
  for (uint8_t i = 0; i < 52; i++) {
    msgBuffer[i] = ' ';
  }

  // Initialise the VL53L0X
  statInfo_t_VL53L0X distanceStr;
  initVL53L0X(1, &hi2c2);

  // Configure the sensor for high accuracy and speed in 20 cm.
  startContinuous(20);

  sprintf(msgBuffer, "start");
  HAL_UART_Transmit(&huart3, (uint8_t*)msgBuffer, strlen(msgBuffer), 50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    // mecanum_drive(0.0f, 0.0f, 20.0f);

    if (forward_flag == 1) {
      if (vx_value > -50.0f) {
        vx_value -= 1.0f;
        if (vx_value < -50.0f) {
          forward_flag = 0;
          vx_value     = -50.0f;
        }
      }
      mecanum_drive(vx_value, 0.0f, 0.0f); // 속도 유지 두번째 -방향 = left turn
      HAL_Delay(20);                       // 부드럽게 올리기 위해 딜레이

    } else if (backward_flag == 1) {
      if (vx_value < +50.0f) {
        vx_value += 1.0f;
        if (vx_value > +50.0f) {
          backward_flag = 0;
          vx_value      = +50.0f;
        }
      }
      mecanum_drive(vx_value, 0.0f, 0.0f); // 속도 유지
      HAL_Delay(20);                       // 부드럽게 올리기 위해 딜레이

    } else if (left_turn_flag == 1) {
      if (vx_value < -10.0f) {
        vx_value += 1.0f; // -50 → -10 으로 점점 증가 (감속)
        if (vx_value > -10.0f) {
          vx_value       = -10.0f; // 최소 속도로 고정
          left_turn_flag = 0;      // 감속 동작 종료
        }
      }
      mecanum_drive(0.0f, vx_value, 0.0f); // -방향 = Left turn
      if (Lifting_check == 0) {
        HAL_Delay(170); // 부드럽게 감속
      } else {
        HAL_Delay(350);
      }

    } else if (right_turn_flag == 1) {
      if (vx_value > 10.0f) {
        vx_value -= 1.0f; // -50 → -10 으로 점점 증가 (감속)
        if (vx_value < 10.0f) {
          vx_value        = 10.0f; // 최소 속도로 고정
          right_turn_flag = 0;     // 감속 동작 종료
        }
      }
      mecanum_drive(0.0f, vx_value, 0.0f); // -방향 = Left turn
      if (Lifting_check == 0) {
        HAL_Delay(170); // 부드럽게 감속
      } else {
        HAL_Delay(350);
      }

    } else if (Linear_flag == 1) {
      Stop();
      Linear_Forward();

      distance    = VL6180X_ReadRange();
      distancestr = readRangeSingleMillimeters(&distanceStr);
      HAL_Delay(20);

      if (distancestr < 150) {
        Stop();
        Linear_Stop();

        sprintf((char*)serialBuf, "9");
        HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

        for (int i = 0; i <= 100; i++) {
          PCA9685_SetServoAngle(0, 120 - i); // 줄어들기
          PCA9685_SetServoAngle(1, 125 + i); // 늘어나기
          PCA9685_SetServoAngle(2, 110 + i); // 늘어나기
          PCA9685_SetServoAngle(3, 113 - i); // 줄어들기
          HAL_Delay(11);
        }
        servo_open = 0;

        HAL_Delay(1000);
        sprintf((char*)serialBuf, "a\r\n");
        HAL_UART_Transmit(&huart3, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);
        HAL_Delay(20);

        sprintf((char*)serialBuf, "%d\r\n", distance);
        HAL_UART_Transmit(&huart3, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

        Lifting_check = 1;
        Linear_flag   = 0;
      }

    } else if (Linear_back_flag == 1) {
      Linear_Backward();

      distance = VL6180X_ReadRange();
      HAL_Delay(20);

      if (distance < 15) {
        Linear_num++;
        if (Linear_num >= 3) {
          Stop();        // 모터 정지 함수
          Linear_Stop(); // 리니어모터 정지 함수 (필요 시)

          sprintf(msgBuffer, "c");
          HAL_UART_Transmit(&huart3, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
          HAL_Delay(20);

          sprintf(msgBuffer, "%d", distance);
          HAL_UART_Transmit(&huart3, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);

          Linear_num       = 0; // 한 번 멈춘 후 다시 카운트 시작
          Linear_back_flag = 0;
          Lifting_check    = 0;
        }
      }

    } else if (servo_close == 1) {
      for (int i = 0; i <= 100; i++) {
        PCA9685_SetServoAngle(0,  20 + i); // 줄어들기
        PCA9685_SetServoAngle(1, 225 - i); // 늘어나기
        PCA9685_SetServoAngle(2, 210 - i); // 늘어나기
        PCA9685_SetServoAngle(3,  13 + i); // 줄어들기
        HAL_Delay(11);
      }

      HAL_Delay(1000);
      servo_close      = 0;
      Linear_back_flag = 1;

    } else if (servo_open == 1) {
      for (int i = 0; i <= 100; i++) {
        PCA9685_SetServoAngle(0, 120 - i); // 줄어들기
        PCA9685_SetServoAngle(1, 125 + i); // 늘어나기
        PCA9685_SetServoAngle(2, 110 + i); // 늘어나기
        PCA9685_SetServoAngle(3, 113 - i); // 줄어들기
        HAL_Delay(11);
      }
      servo_open = 0;
      HAL_Delay(1000);
    }

    // else if(ser == 1)
    // {
    //   for (int i = 0; i <= 70; i++) {
    //     PCA9685_SetServoAngle(0, -20 + i); // 줄어들기
    //     PCA9685_SetServoAngle(1, 195 - i); // 늘어나기
    //     PCA9685_SetServoAngle(2, 180 - i); // 늘어나기
    //     PCA9685_SetServoAngle(3,  45 + i); // 줄어들기
    //     HAL_Delay(10);
    //   }
    // }

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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) { // 잿슨나노 통신
    if (rx_data_3 == '1') {
      if (Lifting_check == 0) {
        backward_flag   = 0;
        forward_flag    = 1;
        right_turn_flag = 0;
        left_turn_flag  = 0;
        vx_value        = 0.0f;

      } else if (Lifting_check == 1) {
        forward_flag    = 0;
        backward_flag   = 0;
        right_turn_flag = 0;
        left_turn_flag  = 0;
        // sprintf((char *)serialBuf, "5");
        // HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
        mecanum_drive(-50.0f, 0.0f, 0.0f);  // 속도 유지
      }

    } else if (rx_data_3 == '2') {
      if (Lifting_check == 0) {
        backward_flag   = 1;
        forward_flag    = 0;
        right_turn_flag = 0;
        left_turn_flag  = 0;
        vx_value        = 0.0f;

      } else if (Lifting_check == 1) {
        forward_flag    = 0;
        backward_flag   = 0;
        right_turn_flag = 0;
        left_turn_flag  = 0;
        // sprintf((char *)serialBuf, "6");
        // HAL_UART_Transmit(&huart6, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
        mecanum_drive(50.0f, 0.0f, 0.0f);  // 속도 유지
      }
      // sprintf((char *)serialBuf, "6"); //Tim4 start
      // HAL_UART_Transmit(&huart7, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);

    } else if (rx_data_3 == '3') {
      sprintf((char*)serialBuf, "3");
      HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

      if (Lifting_check == 1) {
        vx_value = -70.0f;
      } else {
        vx_value = -50.0f;
      }
      left_turn_flag  = 1;
      forward_flag    = 0;
      backward_flag   = 0;
      right_turn_flag = 0;

      // Left_turn();

    } else if (rx_data_3 == '4') {
      sprintf((char*)serialBuf, "4");
      HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

      if (Lifting_check == 1) {
        vx_value = 70.0f;
      } else {
        vx_value = 50.0f;
      }
      left_turn_flag  = 0;
      forward_flag    = 0;
      backward_flag   = 0;
      right_turn_flag = 1;

    } else if (rx_data_3 == '5') {
      Left();

    } else if (rx_data_3 == '6') {
      Right();

    } else if (rx_data_3 == '7') {
      backward_flag = 1;
      sprintf((char*)serialBuf, "1");
      HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

    } else if (rx_data_3 == '8') { // 서보모터 닫기
      servo_close = 1;
      sprintf((char*)serialBuf, "8");
      HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

    } else if (rx_data_3 == 'b') {
      Linear_Backward();

    } else if (rx_data_3 == 'f') {
      Linear_Forward();

    } else if (rx_data_3 == 's') {
      start = 1;

    } else if (rx_data_3 == '9') {
      Linear_back_flag = 0;
      Linear_flag      = 0;
      start            = 0;

      right_turn_flag = 0;
      forward_flag    = 0;
      backward_flag   = 0;
      left_turn_flag  = 0;
      vx_value        = 0.0f;

      Motor_Init();
      Linear_Stop();
      mecanum_drive(0.0f, 0.0f, 0.0f);
      Stop();
    }

    HAL_UART_Receive_IT(&huart3, &rx_data_3, 1);

  } else if (huart->Instance == USART6) { // STM 통신
    if (rx_data_6 == '1') { // 리니어모터정지
      sprintf((char*)serialBuf, "1");
      HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart3, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

    } else if (rx_data_6 == '2') {
      sprintf((char*)serialBuf, "2");
      HAL_UART_Transmit(&huart6, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart3, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

    } else if (rx_data_6 == '3') {
      servo_close = 1;

    } else if (rx_data_6 == '4') {
      sprintf((char*)serialBuf, "s\n");
      HAL_UART_Transmit(&huart3, serialBuf, strlen((char*)serialBuf), HAL_MAX_DELAY);

      Linear_back_flag = 0;
      Linear_flag      = 0;
      right_turn_flag  = 0;
      forward_flag     = 0;
      backward_flag    = 0;
      left_turn_flag   = 0;
      vx_value         = 0.0f;

      Motor_Init();
      Linear_Stop();
      mecanum_drive(0.0f, 0.0f, 0.0f);
      Stop();

    } else if (rx_data_6 == '5') {
      right_turn_flag  = 0;
      forward_flag     = 0;
      backward_flag    = 0;
      left_turn_flag   = 0;
      mecanum_drive(0.0f, 0.0f, 0.0f);

      if (Lifting_check == 1) {
        mecanum_drive(0.0f, -30.0f, 0.0f);
      } else {
        TIM2->CCR1 = 70;
        TIM2->CCR2 = 70;
        TIM2->CCR3 = 70;
        TIM2->CCR4 = 70;
        Left_turn();
      }

    } else if (rx_data_6 == '6') {
      right_turn_flag  = 0;
      forward_flag     = 0;
      backward_flag    = 0;
      left_turn_flag   = 0;
      mecanum_drive(0.0f, 0.0f, 0.0f);

      if (Lifting_check == 1) {
        mecanum_drive(0.0f, 30.0f, 0.0f);
      } else {
        TIM2->CCR1 = 70;
        TIM2->CCR2 = 70;
        TIM2->CCR3 = 70;
        TIM2->CCR4 = 70;
        Right_turn();
      }

    } else if (rx_data_6 == '7') {
      right_turn_flag  = 0;
      forward_flag     = 0;
      backward_flag    = 0;
      left_turn_flag   = 0;
      mecanum_drive(-50.0f, 0.0f, 0.0f);

    } else if (rx_data_6 == '8') {
      forward_flag  = 0;
      backward_flag = 0;
      mecanum_drive(50.0f, 0.0f, 0.0f);

    } else if (rx_data_6 == '9') { // 서보모터 닫기
      backward_flag = 0;
      forward_flag  = 0;
      Linear_flag   = 1;

    } else {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // IN1
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); // IN2
      Stop();
    }

    // 다음 수신 대기
    HAL_UART_Receive_IT(&huart6, &rx_data_6, 1);
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
  while (1) { }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n") */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
