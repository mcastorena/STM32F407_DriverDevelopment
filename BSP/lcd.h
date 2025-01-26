/*
 * lcd.h
 *
 *  Created on: Jan 25, 2025
 *      Author: engineering
 */

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/**
 * Application configurable items
 */
#define LCD_GPIO_PORT			GPIOD
#define LCD_GPIO_RS				GPIO_PIN_NO_0
#define LCD_GPIO_RW				GPIO_PIN_NO_1
#define LCD_GPIO_EN				GPIO_PIN_NO_2
#define LCD_GPIO_D4				GPIO_PIN_NO_3
#define LCD_GPIO_D5				GPIO_PIN_NO_4
#define LCD_GPIO_D6				GPIO_PIN_NO_5
#define LCD_GPIO_D7				GPIO_PIN_NO_6

/**
 * LCD commands
 */
#define LCD_CMD_4DL_2N_5X8F		0x28		// Display configuration: 4 bit data len, 2 limes, 5x8 font
#define LCD_CMD_DON_CURON		0x0E		// Display on, cursor on
#define LCD_CMD_INCADD			0x06		// Increment RAM Address
#define LCD_CMD_DIS_CLEAR		0x01		// Display clear
#define LCD_CMD_DIS_RETURN_HOME	0x02		// Display return home

void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_PrintChar(uint8_t data);
void LCD_PrintString(char* msg);
void LCD_DisplayClear(void);
void LCD_DisplayReturnHome(void);
void LCD_SetCursor(uint8_t row, uint8_t column);


#endif /* LCD_H_ */
