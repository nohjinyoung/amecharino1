/*
 * vl6180x.h
 *
 *  Created on: Aug 12, 2025
 *      Author: noh
 */

#ifndef INC_VL6180X_H_
#define INC_VL6180X_H_

#include "main.h"
#include "i2c.h"
#define VL6180X_DEFAULT_I2C_ADDR    (0x29 << 1)


// 함수 정의
void VL6180X_Init(void);

uint8_t VL6180X_ReadRange(void);
// 내부 사용 I2C 함수 (필요 시 static 처리 가능)
uint8_t  VL6180X_Read8(uint16_t reg);
void     VL6180X_Write8(uint16_t reg, uint8_t value);
uint16_t VL6180X_Read16(uint16_t reg);
void     VL6180X_Write16(uint16_t reg, uint16_t value);

#endif /* INC_VL6180X_H_ */
