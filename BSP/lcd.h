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

void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);

#endif /* LCD_H_ */
