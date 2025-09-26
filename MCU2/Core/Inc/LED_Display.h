/*
 * LED_Display.h
 *
 *  Created on: Sep 26, 2025
 *      Author: noh
 */

#ifndef INC_LED_DISPLAY_H_
#define INC_LED_DISPLAY_H_

#include <main.h>

void tm1637Init(void);
void tm1637DisplayDecimal(int v, int displaySeparator);
void tm1637SetBrightness(char brightness);

#endif /* INC_LED_DISPLAY_H_ */
