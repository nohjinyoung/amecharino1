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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rc522.h"
#include <stdio.h>
#include <string.h>
#include "Anglas_LCD_I2C.h"
#include "LED_Display.h"
#include "step.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 uint8_t status;
 uint8_t str[MAX_LEN]; // Max_LEN = 16
 uint8_t sNum[5];




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 uint32_t adcValue = 0;
 char buff[16];
 typedef struct {
     GPIO_TypeDef* Port;
     uint16_t Pin;
 } Led_t;

 Led_t LEDs[8] = {
     {led_1_GPIO_Port, led_1_Pin},
     {led_2_GPIO_Port, led_2_Pin},
     {led_3_GPIO_Port, led_3_Pin},
     {led_6_GPIO_Port, led_6_Pin},
     {led_7_GPIO_Port, led_7_Pin},
     {led_8_GPIO_Port, led_8_Pin}
 };
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//  tm1637Init();
//  tm1637SetBrightness(8);
//  tm1637DisplayDecimal(1234, 1);


    // Display the value "1234" and turn on the `:` that is between digits 2 and 3.

  ///////////////////////////////////////////////////////////////
  HAL_ADC_Start_IT(&hadc1);

//  LCD_I2C_Init();
//
//  LCD_I2C_Clear();
//  LCD_I2C_WriteText(1, 1, "start");
//  HAL_Delay(1000);

//  int8_t coin = 0;
//  MFRC522_Init();
  /////////////////////////////////////////////////////////////////
//  HAL_TIM_Base_Start(&htim2);
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

//
//  stepCV(256, 1000);  // 반바퀴, 스텝당 2ms (안정적)
//  HAL_Delay(10);
//  stepCCV(128, 1000);  // 반바퀴, 스텝당 2ms (안정적)
  ///////////////////////////////////////////////////////////////////
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  for (int i=0; i<=360; i+=5) {
//	      Stepper_rotate_absolute(5, 10, 0); // CW 1도
//	      HAL_Delay(10);
//	  }
//	  for (int i=0; i<=360; i+=5) {
//	      Stepper_rotate_absolute(5, 10, 1); // CCW 1도
//	      HAL_Delay(10);
//	  }
/////////////////////////////////////////////////////////////////////////
//
//	  	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
//	  		adcValue = HAL_ADC_GetValue(&hadc1);   // 최신 변환 값 읽기
//	  	}
//
//	  int level = (adcValue * 8) / 4096;
//	  for (int i = 0; i < 8; i++) {
//			  if (i < level) {
//				  HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_SET);
//
//			  } else {
//				  HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_RESET);
//
//			  }
//		  }
//
//		  HAL_Delay(50); // 깜박임 방지
	  //////////////////////////////////////////////////////////////////////////////////////////////////
//	  coin++;
//	  sprintf(buff, "coin: %d", coin);
//	  LCD_I2C_Clear();
//	  LCD_I2C_WriteText(1, 1, buff);
//	  HAL_Delay(1000);
////////////////////////////////////////////////////////////////////////////////////
//	    status = MFRC522_Request(PICC_REQIDL, str);
//	    status = MFRC522_Anticoll(str);
//	    memcpy(sNum, str, 5);
//	    HAL_Delay(100);
//	     if((str[0]==179) && (str[1]==158) && (str[2]==54) && (str[3]==250))
//	     {
//	       HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
//	       HAL_Delay(100);
//	       }
//	     else if((str[0]==83) && (str[1]==136) && (str[2]==27) && (str[3]==42))
//	       {
//	       HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
//	       HAL_Delay(2000);
//	     }
//	     else
//	     {
//	       HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
//	     }
///////////////////////////////////////////////////////////////////////////////

//
//	  if(coin == 5)
//	  {
//	      HAL_GPIO_WritePin(led_1_GPIO_Port,led_1_Pin, 1);
//	      HAL_Delay(1000);
//	      HAL_GPIO_WritePin(led_2_GPIO_Port,led_2_Pin, 1);
//	      HAL_Delay(1000);
//	      HAL_GPIO_WritePin(led_3_GPIO_Port,led_3_Pin, 1);
//	      HAL_Delay(1000);
//	      HAL_GPIO_WritePin(led_4_GPIO_Port,led_4_Pin, 1);
//	      HAL_Delay(1000);
//	      HAL_GPIO_WritePin(led_5_GPIO_Port,led_5_Pin, 1);
//	      HAL_Delay(1000);
//	      HAL_GPIO_WritePin(led_6_GPIO_Port,led_6_Pin, 1);
//	      HAL_Delay(1000);
//	      HAL_GPIO_WritePin(led_7_GPIO_Port,led_7_Pin, 1);
//	      HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
    	  uint32_t v = HAL_ADC_GetValue(hadc);   // 최신 ADC 값 읽기
    	    int level = (v * 8) / 4096;            // v로 바로 LED 단계 계산

    	    for (int i = 0; i < 8; i++) {
    	        if (i < level) {
    	            HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_SET);


    	        } else {
    	            HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_RESET);
    	        }
    	    }

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
