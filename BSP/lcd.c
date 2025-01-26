/*
 * lcd.c
 *
 *  Created on: Jan 25, 2025
 *      Author: engineering
 */

#include "lcd.h"

static void writeFourBits(uint8_t value);
static void LCD_Enable(void);

void LCD_Init(void)
{
	/**
	 * Configure LCD GPIO Pins
	 */
	GPIO_Handle_t lcdSignal;
	lcdSignal.pGPIOx = GPIOD;
	lcdSignal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcdSignal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	lcdSignal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcdSignal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	// Register Select pin
	lcdSignal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcdSignal);

	// R/W pin
	lcdSignal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcdSignal);

	// Enable pin
	lcdSignal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcdSignal);

	// D4 pin
	lcdSignal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcdSignal);

	// D5 pin
	lcdSignal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcdSignal);

	// D6 pin
	lcdSignal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcdSignal);

	// D7 pin
	lcdSignal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcdSignal);

	// Init Data Line pins to 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	/**
	 * Initialize the LCD
	 */
	msDelay(40);

	// Set RS and RW to 0, write 0011 on the data lines, wait for 5ms
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	writeFourBits(0x3);
	msDelay(5);

	// Send command 0011, wait for 100us
	writeFourBits(0x3);
	usDelay(150);

	// Send command 0011
	writeFourBits(0x3);

	// Send command 0010
	writeFourBits(0x2);

} // LCD_Init

void LCD_SendCommand(uint8_t cmd)
{
	// Set RS and RW to 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	// Send the higher nibble and transition the enable pin from high to low
	writeFourBits(cmd >> 4);
	LCD_Enable();

	// Send the lower nibble and transition the enable pin from high to low
	writeFourBits(cmd & 0x0F);
	LCD_Enable();
}

void LCD_SendChar(uint8_t data)
{
	// Set RS to 1 and RW to 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	// Send the higher nibble and transition the enable pin from high to low
	writeFourBits(data >> 4);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);

	// Send the lower nibble and transition the enable pin from high to low
	writeFourBits(data & 0x0F);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
} // LCD_SendChar

static void writeFourBits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ( (value >> 0) & 0x1 ));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ( (value >> 1) & 0x1 ));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ( (value >> 2) & 0x1 ));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ( (value >> 3) & 0x1 ));

	LCD_Enable();
} // writeFourBits

static void LCD_Enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	usDelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	usDelay(100);
	/**
	 * Execution time for our application is ~37us
	 * We could instead check the busy flag on D7 but will not for this application
	 */
} // LCD_Enable
