/*
 * step.c
 *
 *  Created on: Sep 25, 2025
 *      Author: noh
 */
#include "step.h"

#define stepsperrev 4096
float currentAngle = 0;

void delay (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000/stepsperrev/rpm);
}

void stepper_step_angle (float angle, int direction, int rpm) //direction-> 0 for CK, 1 for CCK
{
  float anglepersequence = 0.703125;  // 360 = 512 sequences
  int numberofsequences = (int) (angle/anglepersequence);
  for (int seq=0; seq<numberofsequences; seq++)
  {
	if (direction == 0)  // for clockwise
	{
       	  for (int step=7; step>=0; step--)
	  {
	    stepper_half_drive(step);
	    stepper_set_rpm(rpm);
	  }
	}
	else if (direction == 1)  // for anti-clockwise
	{
	  for (int step=0; step<=7; step++)
	  {
	    stepper_half_drive(step);
	    stepper_set_rpm(rpm);
	  }
	}
  }
}

void Stepper_rotate (int angle, int rpm)
{
	int changeinangle = 0;
	changeinangle = angle-currentAngle;  // calculate the angle by which the motor needed to be rotated
	if (changeinangle > 0.71)  // clockwise
	{
		stepper_step_angle (changeinangle,0,rpm);
		currentAngle = angle;  // save the angle as current angle
	}
	else if (changeinangle <0.71) // CCK
	{
		changeinangle = -(changeinangle);
		stepper_step_angle (changeinangle,1,rpm);
		currentAngle = angle;
	}
}

void Stepper_rotate_absolute(float angle, int rpm, int direction) {
    stepper_step_angle(angle, direction, rpm);
}

void stepper_half_drive (int step)
{
  switch (step){
         case 0:
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

	  case 1:
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

          case 2:
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

	  case 3:
		  HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

	  case 4:
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
		  break;

	  case 5:
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
		  break;

	  case 6:
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
		  break;

	  case 7:
		  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
		  break;

	}
}
