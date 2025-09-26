/*
 * step.h
 *
 *  Created on: Sep 25, 2025
 *      Author: noh
 */

#ifndef INC_STEP_H_
#define INC_STEP_H_

#include <main.h>
#include <tim.h>

void delay (uint16_t us);
void Stepper_rotate_absolute(float angle, int rpm, int direction);
void stepper_set_rpm (int rpm);

void stepper_step_angle (float angle, int direction, int rpm);

void Stepper_rotate (int angle, int rpm);

void stepper_half_drive (int step);


#endif /* INC_STEP_H_ */
