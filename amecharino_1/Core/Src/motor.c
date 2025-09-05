/*
 * motor.c
 *
 *  Created on: Aug 12, 2025
 *      Author: noh
 */


#include "motor.h"
#include "tim.h"
#include <math.h>

#define MOTOR1 0  // 앞좌
#define MOTOR2 1  // 앞오
#define MOTOR3 2  // 뒤좌
#define MOTOR4 3  // 뒤오

void Move(int controlcmd){



	switch(controlcmd){
		case FORWARD:
			Forward();
			break;
		case BACKWARD:
			Backward();
			break;
		case RIGHT:
			Right();
			break;
		case LEFT:
			Left();
			break;
		case STOP:
			Stop();
			break;
		default:
			Stop();
			break;
		}
}

void Forward()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);// 2 뒤쪽 좌
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET); //3 앞쪽 우
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET); //4 앞쪽 좌

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //3
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //4

}

void Backward()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);// 2 뒤쪽 좌
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET); //3 앞쪽 우
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET); //4 앞쪽 좌

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //3
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //4
}

void Left()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);// 2 뒤쪽 좌
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET); //3 앞쪽 우
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET); //4 앞쪽 좌

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //3
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //4

	TIM2->CCR1 = 70;
	TIM2->CCR2 = 70;
	TIM2->CCR3 = 70;
	TIM2->CCR4 = 70;
}

void Right()
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);// 2 뒤쪽 좌
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET); //3 앞쪽 우
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET); //4 앞쪽 좌

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //3
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //4

	TIM2->CCR1 = 70;
	TIM2->CCR2 = 70;
	TIM2->CCR3 = 70;
	TIM2->CCR4 = 70;
}

void Right_turn()
{

	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);// 2 뒤쪽 좌
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_SET); //3 앞쪽 우
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET); //4 앞쪽 좌
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void Left_turn()
{


	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);// 2 뒤쪽 좌
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9,GPIO_PIN_RESET); //3 앞쪽 우
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET); //4 앞쪽 좌
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void Stop()
{

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

	TIM2->CCR1 = SPEED;
	TIM2->CCR2 = SPEED;
	TIM2->CCR3 = SPEED;
	TIM2->CCR4 = SPEED;
}

void Motor_Init(){
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);	//BLDC모터 구동 준비
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	TIM2->CCR1 = SPEED;
	TIM2->CCR2 = SPEED;
	TIM2->CCR3 = SPEED;
	TIM2->CCR4 = SPEED;
}

void Linear_Forward(void)
{
	TIM2->CCR1 = 90;
	TIM2->CCR2 = 90;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_RESET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_SET);// 2 뒤쪽 좌
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //2

	TIM9->CCR1 = 40;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // IN1
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); // IN2

}

void Linear_Backward(void)
{
	TIM2->CCR1 = 90;
	TIM2->CCR2 = 90;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,GPIO_PIN_SET); //1 뒤쪽 우
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7,GPIO_PIN_RESET);// 2 뒤쪽 좌
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //2

	TIM9->CCR1 = 40;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);   // IN1
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET); // IN2
}

void Linear_Stop(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);   // IN1
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); // IN2
}

void set_motor_direction(int motor_id, int pwm_val)
{
    GPIO_PinState dir = pwm_val >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (motor_id)
    {
        case MOTOR1:
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, dir);
            break;
        case MOTOR2:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, dir);
            break;
        case MOTOR3:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, dir);
            break;
        case MOTOR4:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, dir);
            break;
    }
}

void mecanum_drive(float Vx, float Vy, float Wz)
{
    int pwm1 = Vy - Vx - Wz;  // 앞좌
    int pwm2 = Vy + Vx + Wz;  // 앞오
    int pwm3 = Vy + Vx - Wz;  // 뒤좌
    int pwm4 = Vy - Vx + Wz;  // 뒤오

    int max_pwm = fmax(fmax(fabs(pwm1), fabs(pwm2)), fmax(fabs(pwm3), fabs(pwm4)));
    if (max_pwm > 100)
    {
        pwm1 = pwm1 * 100 / max_pwm;
        pwm2 = pwm2 * 100 / max_pwm;
        pwm3 = pwm3 * 100 / max_pwm;
        pwm4 = pwm4 * 100 / max_pwm;
    }

    set_motor_direction(MOTOR1, pwm2);
    set_motor_direction(MOTOR2, pwm1);
    set_motor_direction(MOTOR3, pwm3);
    set_motor_direction(MOTOR4, pwm4);

    TIM2->CCR1 = 100 - abs(pwm3);  // 뒤좌
    TIM2->CCR2 = 100 - abs(pwm4);  // 뒤오
    TIM2->CCR3 = 100 - abs(pwm2);  // 앞오
    TIM2->CCR4 = 100 - abs(pwm1);  // 앞좌

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

