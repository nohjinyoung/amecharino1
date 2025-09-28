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
 uint8_t coin = 0;
 uint8_t btn_pressed = 0;	//버튼 누름 여부
 uint32_t pwm_value = 60; 	// 낮을수록 세짐
 uint32_t balance = 10000;   // 카드 초기 금액
 uint8_t used = 0;
 uint16_t step_angle = 200;
 uint8_t level_led = 0;
 uint16_t segment_num = 0;
 float temperature = 0.0f;// 카드 사용 여부

 char lcd_line1[16];
 char lcd_line2[16];
 char buff[16];

 typedef struct {
     GPIO_TypeDef* Port;
     uint16_t Pin;
 } Led_t;

 typedef enum {
     STATE_READY,
     STATE_RUN,
     STATE_FINISH
 } SystemState;

 SystemState sys_state = STATE_FINISH;

 Led_t LEDs[3] = {
     {led_1_GPIO_Port, led_1_Pin},
     {led_2_GPIO_Port, led_2_Pin},
     {led_3_GPIO_Port, led_3_Pin}
 };

 Led_t LEDs_2[3] = {
     {led_6_GPIO_Port, led_6_Pin},
     {led_7_GPIO_Port, led_7_Pin},
     {led_8_GPIO_Port, led_8_Pin}
 };

 void update_leds(uint8_t level)
 {
     for (int i = 0; i < 3; i++) {
         if (i < level)
             HAL_GPIO_WritePin(LEDs_2[i].Port, LEDs_2[i].Pin, GPIO_PIN_SET);
         else
             HAL_GPIO_WritePin(LEDs_2[i].Port, LEDs_2[i].Pin, GPIO_PIN_RESET);
     }
 }

 void update_Lcd()
 {
	 // LCD 갱신
	 if (sys_state == STATE_READY) {
		 sprintf(lcd_line1, " Rest:%lu RDY ", balance);
	 }
	 else if (sys_state == STATE_RUN) {
		 sprintf(lcd_line1, "------RUN------");
		 sprintf(lcd_line2, "               ");
	 }
	 else if (sys_state == STATE_FINISH) {
		 sprintf(lcd_line1, "-----Start-----");
		 sprintf(lcd_line2, "-----Start-----");
	 }
	 LCD_I2C_WriteText(1, 1, lcd_line1);
	 LCD_I2C_WriteText(2, 1, lcd_line2);
 }
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
  MFRC522_Init();
  tm1637Init();
  tm1637SetBrightness(6);
  tm1637DisplayDecimal(segment_num, 1);


    // Display the value "1234" and turn on the `:` that is between digits 2 and 3.

//  ///////////////////////////////////////////////////////////////
  HAL_ADC_Start(&hadc1);
//
  LCD_I2C_Init();

  LCD_I2C_Clear();


  /////////////////////////////////////////////////////////////////
  HAL_TIM_Base_Start(&htim2);
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	//DC모터
//    TIM4 -> CCR4 = pwm_value;

    sprintf(lcd_line1, "Cost:10000 ");
	sprintf(lcd_line2, "Temp:    ");
	 LCD_I2C_WriteText(1, 1, lcd_line1);
	 LCD_I2C_WriteText(2, 1, lcd_line2);


//  for (int i=0; i<=360; i+=5) {
//  	      Stepper_rotate_absolute(5, 10, 0); // CW 1도
//  	      HAL_Delay(10);
//  	  }
//  	  for (int i=0; i<=360; i+=5) {
//  	      Stepper_rotate_absolute(5, 10, 1); // CCW 1도
//  	      HAL_Delay(10);
//  	  }



//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	//DC모터
//  TIM4 -> CCR4 = pwm_value;
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

	  if(sys_state != STATE_RUN)
	  {
			 status = MFRC522_Request(PICC_REQIDL, str);
			 status = MFRC522_Anticoll(str);
			 if (status == MI_OK) {
				 memcpy(sNum, str, 5);
				 balance -= 1000;         // 차감
				 segment_num += 10;
				 tm1637DisplayDecimal(segment_num, 1);
				 sys_state = STATE_READY;   // 동작 상태 변경
				 HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

			 }
	  }

	  if(sys_state == STATE_READY)
	  {
	    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	        adcValue = HAL_ADC_GetValue(&hadc1);
	        float voltage = (3.3f * adcValue) / 4095.0f;
	        int temp_step = (((int)((voltage * 30.3f) + 0.1f)/ 10)) * 10;
	        int level = ( adcValue * 3) / 4096 + 1;  // 0~3 단계
	        if (level > 3) level = 3;

			for (int i = 0; i < 3; i++) {
				if (i < level)
					HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_RESET);


			}

			sprintf(lcd_line2, "Temp:%3dC      ", temp_step);
			LCD_I2C_WriteText(2, 1, lcd_line2);
	    }
	  }
	  else if(sys_state == STATE_RUN)
	  {
		  segment_num -= 1;
		  tm1637DisplayDecimal(segment_num, 1);
		  HAL_Delay(1000);
		  if(!segment_num)
		  {
			  HAL_Delay(1000);
			  sys_state = STATE_FINISH;
		  }

	  }

	  update_Lcd();



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
//		adcValue = HAL_ADC_GetValue(&hadc1);
//		float voltage = (3.3f * adcValue) / 4095.0f;
//		temperature = voltage * 100.0f;  // 센서 보정 필요
//		int level = ( adcValue * 3) / 4096;  // 0~3 단계
//			for (int i = 0; i < 3; i++) {
//				if (i < level)
//					HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(LEDs[i].Port, LEDs[i].Pin, GPIO_PIN_RESET);
//			}
//
//			// LCD 두 번째 줄만 갱신
//			if (sys_state == STATE_RUN)
//				sprintf(lcd_line2, "Temp:%.1fC RUN", temperature);
//			else if (sys_state == STATE_READY)
//				sprintf(lcd_line2, "Temp:%.1fC Rdy", temperature);
//			else if (sys_state == STATE_FINISH)
//				sprintf(lcd_line2, "Temp:%.1fC Fin", temperature);
//
//			LCD_I2C_WriteText(2, 1, lcd_line2);

	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(sys_state == STATE_READY)
	{
	 if (GPIO_Pin == GPIO_PIN_8)
	    {
	        // 버튼이 눌릴 때(상승엣지)마다 실행

		 	 if (level_led < 3) level_led++;   // 최대 3
			 update_leds(level_led);
			 pwm_value -= 10;
			 if (pwm_value <= 40)
				{
					 pwm_value = 40;
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				}
			TIM4->CCR4 = pwm_value;

	    }
	 else if (GPIO_Pin == GPIO_PIN_5)
	 	 {
	 	 	 if (level_led > 0) level_led--;   // 최소 0
			 update_leds(level_led);
			 pwm_value += 10;
			 if (pwm_value >= 60)
				{
					 pwm_value = 60;
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				}
			TIM4->CCR4 = pwm_value;

		}


	 else if (GPIO_Pin == GPIO_PIN_2)
		{
			// 버튼이 눌릴 때(상승엣지)마다 실행
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);


	        if (step_angle > 200) step_angle -= 100;  // 음수 방지
	        else step_angle = 200; // 10도 줄임 (원하는 단위 각도)

	        Stepper_rotate_absolute(5, step_angle, 0); // 모터 위치 이동

		}
	 else if (GPIO_Pin == GPIO_PIN_15)
		{
			// 버튼이 눌릴 때(상승엣지)마다 실행
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		   if (step_angle < 400) step_angle += 100;  // 음수 방지
		   else step_angle = 400; // 10도 줄임 (원하는 단위 각도)

		   Stepper_rotate_absolute(5, step_angle, 0); // 모터 위치 이동
		}
	}

	 if (GPIO_Pin == GPIO_PIN_10)	//동작버튼
		{
			// 버튼이 눌릴 때(상승엣지)마다 실행
			 if(sys_state == STATE_READY)
			 {
				  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	//DC모터
				  TIM4 -> CCR4 = pwm_value;
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				sys_state = STATE_RUN;
			 }

		}
	 else if (GPIO_Pin == GPIO_PIN_11)	//종료 버튼
		{
				// 버튼이 눌릴 때(상승엣지)마다 실행
			 if(sys_state != STATE_FINISH)
			 {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				sys_state = STATE_READY;
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
