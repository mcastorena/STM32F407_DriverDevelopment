/*
 * 001LED_Toggle.c
 *
 *  Created on: Dec 18, 2024
 *      Author: engineering
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	for (uint32_t i = 0; i < 500000; i++);
}

int main(void)
{

	GPIO_Handle_t gpioLed;

	// Set the base address to GPIO Port D
	gpioLed.pGPIOx = GPIOD;

	// Set pin configuration
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;		// The LED we are toggling is the green onboard LED connected to pin 12 on port D
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;	// Set to push-pull output type
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}

	return 0;
}
