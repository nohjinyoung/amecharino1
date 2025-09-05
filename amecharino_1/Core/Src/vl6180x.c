/*
 * vl6180x.c
 *
 *  Created on: Aug 12, 2025
 *      Author: noh
 */


#include "vl6180x.h"
#include "i2c.h" // CubeMX로 생성된 i2c 핸들 사용


#define VL6180X_I2C_ADDR        0x29 << 1  // STM은 8bit 주소 사용

extern I2C_HandleTypeDef hi2c1;  // 사용 중인 I2C 포트에 맞게 수정

uint8_t VL6180X_Read8(uint16_t reg) {
    uint8_t val;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, VL6180X_I2C_ADDR, reg_buf, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, VL6180X_I2C_ADDR, &val, 1, HAL_MAX_DELAY);
    return val;
}

void VL6180X_Write8(uint16_t reg, uint8_t val) {
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};
    HAL_I2C_Master_Transmit(&hi2c1, VL6180X_I2C_ADDR, buf, 3, HAL_MAX_DELAY);
}

uint16_t VL6180X_Read16(uint16_t reg) {
    uint8_t val[2];
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, VL6180X_I2C_ADDR, reg_buf, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, VL6180X_I2C_ADDR, val, 2, HAL_MAX_DELAY);
    return (val[0] << 8) | val[1];
}

void VL6180X_Write16(uint16_t reg, uint16_t val) {
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, VL6180X_I2C_ADDR, buf, 4, HAL_MAX_DELAY);
}

void VL6180X_Init() {
    if (VL6180X_Read8(0x016) == 1) {
        VL6180X_Write8(0x0207, 0x01);
        VL6180X_Write8(0x0208, 0x01);
        VL6180X_Write8(0x0096, 0x00);
        VL6180X_Write8(0x0097, 0xFD);
        VL6180X_Write8(0x00E3, 0x00);
        VL6180X_Write8(0x00E4, 0x04);
        VL6180X_Write8(0x00E5, 0x02);
        VL6180X_Write8(0x00E6, 0x01);
        VL6180X_Write8(0x00E7, 0x03);
        VL6180X_Write8(0x00F5, 0x02);
        VL6180X_Write8(0x00D9, 0x05);
        VL6180X_Write8(0x00DB, 0xCE);
        VL6180X_Write8(0x00DC, 0x03);
        VL6180X_Write8(0x00DD, 0xF8);
        VL6180X_Write8(0x009F, 0x00);
        VL6180X_Write8(0x00A3, 0x3C);
        VL6180X_Write8(0x00B7, 0x00);
        VL6180X_Write8(0x00BB, 0x3C);
        VL6180X_Write8(0x00B2, 0x09);
        VL6180X_Write8(0x00CA, 0x09);
        VL6180X_Write8(0x0198, 0x01);
        VL6180X_Write8(0x01B0, 0x17);
        VL6180X_Write8(0x01AD, 0x00);
        VL6180X_Write8(0x00FF, 0x05);
        VL6180X_Write8(0x0100, 0x05);
        VL6180X_Write8(0x0199, 0x05);
        VL6180X_Write8(0x01A6, 0x1B);
        VL6180X_Write8(0x01AC, 0x3E);
        VL6180X_Write8(0x01A7, 0x1F);
        VL6180X_Write8(0x0030, 0x00);
        VL6180X_Write8(0x10A, 0x30);
        VL6180X_Write8(0x03F, 0x46);
        VL6180X_Write8(0x031, 0xFF);
        VL6180X_Write16(0x040, 0x0063);
        VL6180X_Write8(0x02E, 0x01);
        VL6180X_Write8(0x01B, 0x09);
        VL6180X_Write8(0x03E, 0x31);
        VL6180X_Write8(0x014, 0x00);
        VL6180X_Write8(0x01C, 0x31);
        VL6180X_Write8(0x2A3, 0x00);
        VL6180X_Write8(0x011, 0x20);
        VL6180X_Write8(0x016, 0x00);
    }
}

uint8_t VL6180X_ReadRange() {
    VL6180X_Write8(0x018, 0x01);  // Trigger single range measurement
    HAL_Delay(10);  // Wait for conversion
    return VL6180X_Read8(0x062);  // Result
}
