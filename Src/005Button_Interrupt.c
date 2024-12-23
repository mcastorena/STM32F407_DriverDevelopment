/*
 * 005Button_Interrupt.c
 *
 *  Created on: Dec 22, 2024
 *      Author: engineering
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	// Driver layer handling
	GPIO_IRQHandling(GPIO_PIN_NO_6);

	// Application layer handling
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
} // EXTI9_5_IRQHandler


int main(void)
{

	GPIO_Handle_t gpioLed;
	memset(&gpioLed, 0, sizeof(gpioLed));

	// Set the base address to GPIO Port D
	gpioLed.pGPIOx = GPIOD;

	// Set pin configuration
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;		// The LED we are toggling is the green onboard LED connected to pin 12 on port D
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	// Set to push-pull output type
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(gpioLed.pGPIOx, ENABLE);
	GPIO_Init(&gpioLed);
	GPIO_WriteToOutputPin(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);

	GPIO_Handle_t gpioButton;
	memset(&gpioButton, 0, sizeof(gpioButton));

	// Set base address to GPIO port D
	gpioButton.pGPIOx = GPIOD;

	// Set pin configuration
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;	// Interrupt mode with falling edge trigger
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;	// We will be using an external 22k Ohm pull-up resistor

	// Enable peripheral clock
	GPIO_PeriClockControl(gpioButton.pGPIOx, ENABLE);
	GPIO_Init(&gpioButton);


	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXT9_5, NVIC_IRQ_PRI15);		// Setting priority is optional
	GPIO_IRQInterruptConfig(IRQ_NO_EXT9_5, ENABLE);

	while(1);

	return 0;
}
