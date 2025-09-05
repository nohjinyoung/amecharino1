/*
 * motor.h
 *
 *  Created on: Aug 12, 2025
 *      Author: noh
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "tim.h"

#define SPEED 50

typedef enum {
	STOP,
	FORWARD,
	RIGHT,
	BACKWARD,
	LEFT,
	CW,
	CCW
}CONTROLLER_SIGNAL;

void Move(int controlcmd);
void Forward();
void Backward();
void Right();
void Left();
void Right_turn();
void Left_turn();
void Stop();
void Motor_Init();
void Linear_Forward();
void Linear_Backward();
void Linear_Stop();
void set_motor_direction(int motor_id, int pwm_val);
void mecanum_drive(float Vx, float Vy, float Wz);
#endif /* INC_MOTOR_H_ */
