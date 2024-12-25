/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Dec 23, 2024
 *      Author: engineering
 */
#include "stm32f407xx_spi_driver.h"

/**
 * Enable/Disable peripheral clock for the given SPI peripheral
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	    {
	        if (pSPIx == SPI1)
	        {
	            SPI1_PCLK_EN();
	        }
	        else if (pSPIx == SPI2)
	        {
	            SPI2_PCLK_EN();
	        }
	        else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
	        else
	        {
	            // Handle invalid GPIO port
	        }
	    }
	    else if (EnorDi == DISABLE)
	    {
	    	if (pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
	        else
	        {
	            // Handle invalid GPIO port
	        }
	    }
}


/**
 * Initialize the given SPI peripheral
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/**
	 * Configure SPI_CR1
	 */
	uint32_t tmpReg = 0;

	// Configure device mode
	tmpReg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// Configure bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDIMODE should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tmpReg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDIMODE should be cleared and RXONLY should be set
		tmpReg &= ~(1 << SPI_CR1_BIDI_MODE);
		tmpReg |= (1 << SPI_CR1_RX_ONLY);
	}

	// Configure SCLK speed
	tmpReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// Configure data frame format
	tmpReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// Configure clock polarization
	tmpReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Configure clock phase
	tmpReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// Configure slave select mode
	tmpReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	// Enable the peripheral
	tmpReg |= (ENABLE << SPI_CR1_SPE);

	// Write to the SPI peripheral register
	*(pSPIHandle->pSPIx->CR1) = tempReg;

} // SPI_Init

/**
 * De-initialize the given SPI peripheral
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	/**
	 * @Note More steps should be followed for safe de-initialization
	 * Refer to Section 28.3.8 in the Reference Manual
	 */
	// Disable the SPE bit in the CR1 register
	pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
} // SPI_DeInit

/**
 * Retrieves flag status from the SPI_SR register
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName)
{
	uint8_t flagStatus = FLAG_RESET;

	if( pSPIx->SR & flagName)
	{
		flagStatus = FLAG_SET;
	}

	return flagStatus;
} // SPI_GetStatusFlag

/**
 * Send data
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		// Wait until the TX Buffer is empty by checking TXE in the status register
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == FLAG_RESET);

		// Check the DFF bit in SPI_CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);	// Load 16bits of data into the DR
			len -= 2;								// Decrement len by 2 (bytes)
			(uint16_t*)pTxBuffer++;					// Increment the pointer by 2 bytes
		}
		else
		{
			// 8 BIT DFF
			pSPIx->DR = *(pTxBuffer);				// Load 8bits of data into the DR
			len--;									// Decrement len by 1(byte)
			pTxBuffer++;							// Increment the pointer by 1 byte
		}
	}
} // SPI_SendData

/**
 * @brief
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);


/**
 * Enable or disable the given IRQ number
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * Set the IRQ priority level for the given IRQ number
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * Handle an interrupt for the SPI peripheral
 */
void SPI_IRQHandling(SPI_RegDef_t *pSPIx);

