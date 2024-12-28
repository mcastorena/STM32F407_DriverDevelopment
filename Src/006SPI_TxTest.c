/*
 * 006SPI_TxTest.c
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

void SPI2_Init(void)
{
	// Create SPI Handle for SPI2
	SPI_Handle_t SPI2Handle;
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		// 8MHz with default settings using HSI
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;						// We have no slaves in this application

	// Initialize SPI2
	SPI_Init(&SPI2Handle);
}

int main(void)
{

	// Initialize the GPIO pins to behave as SPI2 Pins
	SPI_GPIOInit(GPIOB, SPI2_AF_MODE, GPIO_PIN_NO_13, GPIO_PIN_NO_12, GPIO_PIN_NO_14, GPIO_PIN_NO_12);

	// Initialize the SPI2 Peripheral
	SPI2_Init();

	// Enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// Send data
	char userDataBuffer[] = "Hello World!";
	SPI_SendData(SPI2, (uint8_t*)userDataBuffer, strlen(userDataBuffer));

	// Confirm the SPI peripheral is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)); 		// SPI is busy when this returns 1

	// Disable the SPI2 Peripheral
	SPI_PeriClockControl(SPI2, DISABLE);

	while(1);



	return 0;
}
