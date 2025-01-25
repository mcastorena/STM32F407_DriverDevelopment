/*
 * 015UART_Tx.c
 *
 *  Created on: Jan 20, 2025
 *      Author: engineering
 */
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"

#define MAX_LEN			1024
#define USART2_AF_MODE	7

char msg[MAX_LEN] = "UART Tx test...\n\r";

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioLed;

	// Set the base address to GPIO Port D
	gpioLed.pGPIOx = GPIOD;

	// Set pin configuration
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;		// The LED we are toggling is the green onboard LED connected to pin 12 on port D
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	// Set to push-pull output type
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioLed);

	GPIO_Handle_t gpioButton;

	// Set base address to GPIO port A
	gpioButton.pGPIOx = GPIOA;

	// Set pin configuration
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&gpioButton);
}

void waitForButtonInput()
{
	// Wait until the button is pressed
	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

	// Delay to avoid the button debouncing
	delay();

	// Toggle onboard LED for debugging
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

USART_Handle_t USART2Handle;

void USART2_Init(void)
{
	// Init GPIO pins
	USART_GPIOInit(GPIOA, USART2_AF_MODE, GPIO_PIN_NO_3, GPIO_PIN_NO_2);

	// Configure the handle struct
	memset(&USART2Handle, 0, sizeof(USART2Handle));
	USART2Handle.pUSARTx = USART2;
	USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&USART2Handle);
}

int main(void)
{
	// Initialize the User button and onboard LED
	GPIO_ButtonInit();

	// Init USART2 Peripheral and enable
	USART2_Init();
	USART_PeripheralControl(USART2Handle.pUSARTx, ENABLE);

	while(1)
	{
		waitForButtonInput();
		USART_SendData(&USART2Handle, (uint8_t *)msg, strlen(msg));
	}

	return 0;
}
