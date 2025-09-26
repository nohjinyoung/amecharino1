/*
 * Anglas_LCD_I2C.h
 *
 *  Created on: Sep 23, 2025
 *      Author: noh
 */

#ifndef INC_ANGLAS_LCD_I2C_H_
#define INC_ANGLAS_LCD_I2C_H_


// P7 P6 P5 P4 P3 P2 P1 P0 (PCF8574)
// D7 D6 D5 D4 K  E  RW RS (LCD16X2)

// RS: REGISTER SELECT
// RS = 0 (IR:Instruccion Register) - Registro para configuracion de pantalla
// RS = 1 (DR:Data Register)        - Registro para acceder alas memorias RAM ROM

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

/* Exported constant ---------------------------------------------------------*/
#define PCF8574_ADDRESS (0x27 << 1) //0b01001110
//#define PCF8574_ADDRESS 0x40 //0b01000000

/* 제어 비트 */
#define PIN_RS    0x01
#define PIN_EN    0x04
#define BACKLIGHT 0x08

/* LCD 명령어 */
#define DISPLAY_CLEAR   0x01
#define MODE_4_BITS     0x28
#define DISPLAY_ON      0x0C
#define ENTRY_MODE_SET  0x06


void LCD_I2C_Init(void);
void LCD_I2C_Clear(void);
void LCD_I2C_WriteText(uint8_t row, uint8_t col, char *str);
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Variables -----------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void LCD_I2C_Init(void);
void LCD_I2C_ClearText(void);
void LCD_I2C_Set_Cursor_Blink(uint8_t row, uint8_t col, uint8_t blink);
void LCD_I2C_WriteText(uint8_t row, uint8_t col, char* string);

void LCD_I2C_Shift_Right(void);
void LCD_I2C_Shift_Left(void);
void LCD_I2C_CGRAM_CreateChar(uint8_t pos, const uint8_t *msg);
void LCD_I2C_CGRAM_WriteChar(uint8_t row, uint8_t col, uint8_t pos);

#endif /* INC_ANGLAS_LCD_I2C_H_ */
