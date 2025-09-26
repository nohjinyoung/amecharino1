/*
 * Anglas_LCD_I2C.c
 *
 *  Created on: Sep 23, 2025
 *      Author: noh
 */


#include "main.h"
#include "Anglas_LCD_I2C.h"


static void LCD_I2C_WriteCommand(uint8_t cmd) {
    uint8_t high = cmd & 0xF0;
    uint8_t low  = (cmd << 4) & 0xF0;
    uint8_t data[4];

    data[0] = high | BACKLIGHT | PIN_EN;
    data[1] = high | BACKLIGHT;
    data[2] = low  | BACKLIGHT | PIN_EN;
    data[3] = low  | BACKLIGHT;

    HAL_I2C_Master_Transmit(&hi2c1, PCF8574_ADDRESS, data, 4, 100);
    HAL_Delay(2);
}

static void LCD_I2C_WriteData(uint8_t value) {
    uint8_t high = value & 0xF0;
    uint8_t low  = (value << 4) & 0xF0;
    uint8_t data[4];

    data[0] = high | BACKLIGHT | PIN_EN | PIN_RS;
    data[1] = high | BACKLIGHT | PIN_RS;
    data[2] = low  | BACKLIGHT | PIN_EN | PIN_RS;
    data[3] = low  | BACKLIGHT | PIN_RS;

    HAL_I2C_Master_Transmit(&hi2c1, PCF8574_ADDRESS, data, 4, 100);
    HAL_Delay(2);
}

void LCD_I2C_Init(void) {
    HAL_Delay(50);
    LCD_I2C_WriteCommand(0x33); // 초기화 시퀀스
    LCD_I2C_WriteCommand(0x32); // 4비트 모드
    LCD_I2C_WriteCommand(MODE_4_BITS);
    LCD_I2C_WriteCommand(DISPLAY_ON);
    LCD_I2C_WriteCommand(ENTRY_MODE_SET);
    LCD_I2C_Clear();
}

void LCD_I2C_Clear(void) {
    LCD_I2C_WriteCommand(DISPLAY_CLEAR);
    HAL_Delay(2);
}

void LCD_I2C_WriteText(uint8_t row, uint8_t col, char *str) {
    uint8_t address = (row == 1) ? 0x80 : 0xC0;
    address += (col - 1);
    LCD_I2C_WriteCommand(address);

    while (*str) {
        LCD_I2C_WriteData(*str++);
    }
}
