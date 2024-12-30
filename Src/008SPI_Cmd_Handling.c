/*
 * 008SPI_Cmd_Handling.c
 *
 *  Created on: Dec 29, 2024
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
#include <stdio.h>

extern void initialise_monitor_handles();

/**
 * Command codes
 */
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

/**
 * Slave response codes
 */
#define ACK						0xF5
#define NACK					0xA5

/**
 * Arduino analog pins
 */
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

// Arduino LED
#define LED_PIN					9


#define SPI2_AF_MODE		5

char userDataBuffer[] = "Hello World!";

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

uint8_t SPI_VerifyReponse(uint8_t ackByte)
{
	if(ackByte == ACK)
	{
		return 1;
	}
	return 0;
}

int main(void)
{
	// SPI Dummy Data
	uint8_t dummyWrite = 0xFF;
	uint8_t dummyRead = 0;

	// Initialize to use printf
	initialise_monitor_handles();
	printf("Application is running!\n");

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
		// Wait until the button is pressed
		waitForButtonInput();

		// Enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);
		printf("SPI peripheral enabled!\n");

		/**
		 * 1. CMD_LED_CTRL		<pin no (1)>		<value (1)>
		 */
		uint8_t cmdCode	=	COMMAND_LED_CTRL;
		uint8_t ackByte = 0;
		uint8_t args[2];
		SPI_SendData(SPI2, &cmdCode, 1);		// Send the command
		SPI_ReceiveData(SPI2, &dummyRead, 1);	// Perform a dummy read to clear the RX Buffer
		SPI_SendData(SPI2, &dummyWrite, 1);		// Send some dummy bits to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackByte, 1);		// Read slave acknowledgment response data
		if(SPI_VerifyReponse(ackByte))			// Verify the response
		{
			// Send the command arguments if a ACK was received
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			printf("LED ON command sent.\n");
		} // End of CMD_LED_CTRL

		// Wait until the button is pressed
		waitForButtonInput();

		/**
		 * 2. CMD_SENSOR_READ	<analog pin no(1)>
		 */
		cmdCode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &cmdCode, 1);		// Send the command
		SPI_ReceiveData(SPI2, &dummyRead, 1);	// Perform a dummy read to clear the RX Buffer
		SPI_SendData(SPI2, &dummyWrite, 1);		// Send some dummy bits to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackByte, 1);		// Read slave acknowledgment response data
		if(SPI_VerifyReponse(ackByte))			// Verify the response
		{
			// Send the command arguments if a ACK was received
			args[0] = ANALOG_PIN5;
			SPI_SendData(SPI2, args, 1);	// Send only one byte of data
			SPI_ReceiveData(SPI2, &dummyRead, 1);	// Perform a dummy read to clear the RX Buffer

			delay();								// Insert delay to allow the slave to fetch the data
			SPI_SendData(SPI2, &dummyWrite, 1);		// Send some dummy bits to fetch the response from the slave

			uint8_t analogRead = 0;
			SPI_ReceiveData(SPI2, &analogRead, 1);		// Read slave response sensor read data
			printf("Received analog sensor reading: %d\n", analogRead);
		} // End of CMD_SENSOR_READ

		// Wait until the button is pressed
		waitForButtonInput();

		/**
		 * 3. CMD_LED_READ	<pin no (1)>
		 */
		cmdCode = COMMAND_LED_READ;
		SPI_SendData(SPI2, &cmdCode, 1);		// Send the command
		SPI_ReceiveData(SPI2, &dummyRead, 1);	// Perform a dummy read to clear the RX Buffer
		SPI_SendData(SPI2, &dummyWrite, 1);		// Send some dummy bits to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackByte, 1);		// Read slave acknowledgment response data
		if(SPI_VerifyReponse(ackByte))			// Verify the response
		{
			// Send the command arguments if a ACK was received
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);	// Send only one byte of data
			SPI_ReceiveData(SPI2, &dummyRead, 1);	// Perform a dummy read to clear the RX Buffer

			delay();								// Insert delay to allow the slave to fetch the data
			SPI_SendData(SPI2, &dummyWrite, 1);		// Send some dummy bits to fetch the response from the slave

			uint8_t ledRead = 0;
			SPI_ReceiveData(SPI2, &ledRead, 1);		// Read slave response sensor read data
			printf("Received LED pin reading: %d\n", ledRead);
		} // End of CMD_LED_READ

		// Wait until the button is pressed
		waitForButtonInput();


		/**
		 * 4. CMD_PRINT		<len(1)>	<message(1)>
		 */
		cmdCode = COMMAND_PRINT;
		SPI_SendData(SPI2, &cmdCode, 1);		// Send the command
		SPI_ReceiveData(SPI2, &dummyRead, 1);	// Perform a dummy read to clear the RX Buffer
		SPI_SendData(SPI2, &dummyWrite, 1);		// Send some dummy bits to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackByte, 1);		// Read slave acknowledgment response data
		if(SPI_VerifyReponse(ackByte))			// Verify the response
		{
			// Send the command arguments if a ACK was received
			uint8_t msgBuffer[] = "STM32 Master print command!!!";
			args[0] = strlen((char*)msgBuffer);
			SPI_SendData(SPI2, args, 1);				// Send length
			SPI_SendData(SPI2, msgBuffer, args[0]);		// Send message

			printf("Print command executed. Message: %s\n", msgBuffer);
		} // End of CMD_PRINT

		// Wait until the button is pressed
		waitForButtonInput();


		/**
		 * 5. CMD_ID_READ
		 */
		uint8_t deviceID[11];					// Device ID message buffer
		cmdCode = COMMAND_ID_READ;
		SPI_SendData(SPI2, &cmdCode, 1);		// Send the command
		SPI_ReceiveData(SPI2, &dummyRead, 1);	// Perform a dummy read to clear the RX Buffer
		SPI_SendData(SPI2, &dummyWrite, 1);		// Send some dummy bits to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackByte, 1);		// Read slave acknowledgment response data
		if(SPI_VerifyReponse(ackByte))			// Verify the response
		{
			// Read the 10byte ID from the slave
			for(uint32_t i = 0; i < 10; i++)
			{
				SPI_SendData(SPI2, &dummyWrite, 1);			// Send some dummy bits to fetch the response from the slave
				SPI_ReceiveData(SPI2, &deviceID[i], 1);		// Read in one byte from the ID message
			}
			printf("Device ID received: %s\n", deviceID);
		} // End of CMD_ID_READ

		// Confirm the SPI peripheral is not busy before disabling the peripheral
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)); 		// SPI is busy when this returns 1

		// Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
		printf("SPI peripheral disabled!\n");

	}


	return 0;
}
