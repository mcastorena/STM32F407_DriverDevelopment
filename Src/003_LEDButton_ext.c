/*
 * 002_LEDButton.c
 *
 *  Created on: Dec 19, 2024
 *      Author: engineering
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{

	GPIO_Handle_t gpioLed;

	// Set the base address to GPIO Port A
	gpioLed.pGPIOx = GPIOA;

	// Set pin configuration
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;		// The LED we are toggling is the green onboard LED connected to pin 12 on port D
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	// Set to push-pull output type
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(gpioLed.pGPIOx, ENABLE);
	GPIO_Init(&gpioLed);

	GPIO_Handle_t gpioButton;

	// Set base address to GPIO port B
	gpioButton.pGPIOx = GPIOB;

	// Set pin configuration
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Enable peripheral clock
	GPIO_PeriClockControl(gpioButton.pGPIOx, ENABLE);
	GPIO_Init(&gpioButton);


	while(1)
	{
		if(!GPIO_ReadFromInputPin(gpioButton.pGPIOx, gpioButton.GPIO_PinConfig.GPIO_PinNumber))
		{
			delay();
			GPIO_ToggleOutputPin(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber);
		}

	}

	return 0;
}
