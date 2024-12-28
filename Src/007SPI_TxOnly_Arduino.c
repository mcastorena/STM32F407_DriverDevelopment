/*
 * 007SPI_TxOnly_Arduino.c
 *
 *  Created on: Dec 25, 2024
 *      Author: engineering
 */

/**
 * PB12 --> SPI2_MISO
 * PB13 --> SPI2_MOSI
 * PB14 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 5
 */
#include "stm32f407xx_spi_driver.h"
#include <string.h>

#define SPI2_AF_MODE		5

char userDataBuffer[] = "Hello World!";


void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

void SPI2_Init(void)
{
	// Create SPI Handle for SPI2
	SPI_Handle_t SPI2Handle;
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;		// Generates SCLK of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;						// We be using hardware slave management

	// Initialize SPI2
	SPI_Init(&SPI2Handle);
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

int main(void)
{
	// Initialize the onboard user button
	GPIO_ButtonInit();

	// Initialize the GPIO pins to behave as SPI2 Pins
	SPI_GPIOInit(GPIOB, SPI2_AF_MODE, GPIO_PIN_NO_15, GPIO_PIN_NO_14, GPIO_PIN_NO_13, GPIO_PIN_NO_12);

	// Initialize the SPI2 Peripheral
	SPI2_Init();

	// Enable SSOE
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

			// Enable the SPI2 Peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			// Send length information
			uint8_t dataLen = strlen(userDataBuffer);
			SPI_SendData(SPI2, &dataLen, 1);

			// Send data
			SPI_SendData(SPI2, (uint8_t*)userDataBuffer, strlen(userDataBuffer));

			// Confirm the SPI peripheral is not busy
			while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)); 		// SPI is busy when this returns 1

			// Disable the SPI2 Peripheral
			SPI_PeripheralControl(SPI2, DISABLE);
		}

	}


	return 0;
}
