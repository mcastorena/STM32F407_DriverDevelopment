/*
 * 009SPI_Message_Rcv_IT.c
 *
 *  Created on: Dec 30, 2024
 *      Author: engineering
 */

#include "stm32f407xx_spi_driver.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();


#define MAX_LEN		500
#define SPI2_AF_MODE		5


char rcvBuffer[MAX_LEN];

char readByte = 'a';

volatile uint8_t rcvStop;

// This flag will be set in the interrupt handler of the Ardunio interrupt GPIO
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

GPIO_Handle_t gpioIT;
void Slave_GPIO_InterruptPinInit(void)
{
	memset(&gpioIT, 0, sizeof(gpioIT));

	// Set the base address to GPIO Port D
	gpioIT.pGPIOx = GPIOD;

	// Set pin configuration
	gpioIT.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpioIT.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;		// Arduino will pull the pin low to trigger an interrupt
	gpioIT.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioIT.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

	GPIO_Init(&gpioIT);

	// Set IRQ config
	GPIO_IRQPriorityConfig(IRQ_NO_EXT9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXT9_5, ENABLE);
}

/**
 * PB12 --> SPI2_MISO
 * PB13 --> SPI2_MOSI
 * PB14 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 5
 */
SPI_Handle_t SPI2Handle;
void SPI2_Init(void)
{
	// Initialize the GPIO pins to behave as SPI2 Pins
	SPI_GPIOInit(GPIOB, SPI2_AF_MODE, GPIO_PIN_NO_15, GPIO_PIN_NO_14, GPIO_PIN_NO_13, GPIO_PIN_NO_12);

	// Initialize to 0
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;		// Generates SCLK of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;						// We will be using hardware slave management

	// Initialize SPI2
	SPI_Init(&SPI2Handle);

	// Enable SSOE
	SPI_SSOEConfig(SPI2, ENABLE);

	// Initialize IRQ config
	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);
}

// IRQ Handler function implementations
/**
 * Runs when a data byte is received from the peripheral over SPI
 */
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

/**
 * Slave data available interrupt handler
 */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_7);

	// Set the data available flag
	dataAvailable = 1;
}

/**
 * Application event call-back override function
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv)
{
	static uint32_t i = 0;

	if(appEv == SPI_EVENT_RX_COMPLETE)
	{
		// When we receive the SPI_EVENT_RX_COMPLETE notification, copy the data into the receive buffer
		rcvBuffer[i++] = readByte;

		// Stop receiving data when we receive the null-terminator or the receive buffer is full
		if( (readByte == '\0') || (i == MAX_LEN))
		{
			rcvStop = 1;
			rcvBuffer[i-1] = '\0';		// Insert null-terminator character
			i = 0;						// Reset position index
		}
	}
}


int main(void)
{
	// SPI Dummy Data
	char dummyWrite = 'a';

	// Initialize to use printf
	initialise_monitor_handles();
	printf("Application is running!\n");

	// Initialize the slave interrupt pin
	Slave_GPIO_InterruptPinInit();

	// Initialize the SPI2 peripheral
	SPI2_Init();

	while(1)
	{
		rcvStop = 0;

		// Wait for the data available interrupt from the Slave
		while(!dataAvailable);

		// Disable the interrupt pin while we receive data
		GPIO_IRQInterruptConfig(IRQ_NO_EXT9_5, DISABLE);

		// Enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// Read in the data
		while(!rcvStop)
		{
			// Fetch the data from SPI2 one byte at a time in interrupt mode
			// This will retry until there is no on-going transmission
			while( SPI_SendDataIT(&SPI2Handle, &dummyWrite, 1) == SPI_BUSY_IN_TX );		// We must send one byte of data to receive one byte of data in full duplex mode

			// This will retry until there is no on-going reception
			while( SPI_ReceiveDataIT(&SPI2Handle, &readByte, 1) == SPI_BUSY_IN_RX );
		}

		// Confirm the SPI peripheral is not busy before disabling the peripheral
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)); 		// SPI is busy when this returns 1

		// Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		// Print message
		printf("Received message: %s", rcvBuffer);

		// Reset data available flag
		dataAvailable = 0;

		// Enable the interrupt pin now that we have finished receiving and processing the data
		GPIO_IRQInterruptConfig(IRQ_NO_EXT9_5, ENABLE);
	}

	return 0;
}
