/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Dec 23, 2024
 *      Author: engineering
 */
#include "stm32f407xx_spi_driver.h"
#include <string.h>

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
	// Enable the SPI peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

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
		tmpReg &= ~(1 << SPI_CR1_BIDI_MODE);
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

	// Configure internal slave select
	if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
	{
		/**
		 * SSI bit must be set HIGH if software slave management is enabled
		 */
		tmpReg |= (1 << SPI_CR1_SSI);
	}

	// Write to the SPI peripheral register
	pSPIHandle->pSPIx->CR1 = tmpReg;

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
 * Initializes the GPIO pins for SPI usage
 */
void SPI_GPIOInit(GPIO_RegDef_t *pGPIOx, uint8_t AFMode, uint8_t MOSIPin, uint8_t MISOPin, uint8_t SCLKPin, uint8_t NSSPin)
{
	// Create and initialize GPIO Handle
	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins));
	SPIPins.pGPIOx = pGPIOx;

	// Set general pin configuration
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = AFMode;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	// Push-pull output type
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// Init SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = SCLKPin;
	GPIO_Init(&SPIPins);

	// Init MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = MISOPin;
	GPIO_Init(&SPIPins);

	// Init MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = MOSIPin;
	GPIO_Init(&SPIPins);

	// Init NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = NSSPin;
	GPIO_Init(&SPIPins);
} // SPI_GPIOInit

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
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

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
 * @brief Read Data
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while(len > 0)
		{
			// Wait until the RX Buffer is not empty by checking RXNE in the status register
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			// Check the DFF bit in SPI_CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				// 16 bit DFF
				*((uint16_t*)pRxBuffer) = pSPIx->DR;	// Read 16bits of data from the DR
				len -= 2;								// Decrement len by 2 (bytes)
				(uint16_t*)pRxBuffer++;					// Increment the pointer by 2 bytes
			}
			else
			{
				// 8 BIT DFF
				*(pRxBuffer) = pSPIx->DR;				// Read 8bits of data from the DR
				len--;									// Decrement len by 1(byte)
				pRxBuffer++;							// Increment the pointer by 1 byte
			}
		}
} // SPI_ReceiveData

/**
 * Interrupt-based send data
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txState = pSPIHandle->txState;

	if(txState != SPI_BUSY_IN_TX)
	{
		// Save TX Buffer address and len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = len;

		// Mark SPI TX state as busy in transmission so no other code can take over control of the peripheral until
		// transmission is complete
		pSPIHandle->txState = SPI_BUSY_IN_TX;

		// Enable the TXEIE control bit (in SPI_CR2) to get an interrupt whenever the TXE flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

		// Data transmission will be handled by the ISR
	}

	return txState;
} // SPI_SendDataIT

/**
 * Interrupt-based read data
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t rxState = pSPIHandle->rxState;

		if(rxState != SPI_BUSY_IN_RX)
		{
			// Save RX Buffer address and len information in global variables
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->rxLen = len;

			// Mark SPI RX state as busy in reception so no other code can take over control of the peripheral until
			// reception is complete
			pSPIHandle->rxState = SPI_BUSY_IN_RX;

			// Enable the RXNEIE control bit (in SPI_CR2) to get an interrupt whenever the TXE flag is set in the SR
			pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

			// Data transmission will be handled by the ISR
		}

		return rxState;
} // SPI_ReadDataIT

/**
 * SPI ISR Helper functions
 */
static void SPI_TXEInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in SPI_CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);	// Load 16bits of data into the DR
		pSPIHandle->txLen -= 2;											// Decrement len by 2 (bytes)
		(uint16_t*)pSPIHandle->pTxBuffer++;								// Increment the pointer by 2 bytes
	}
	else
	{
		// 8 BIT DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);				// Load 8bits of data into the DR
		pSPIHandle->txLen--;											// Decrement len by 1 (byte)
		pSPIHandle->pTxBuffer++;										// Increment the pointer by 1 byte
	}

	// If the SPI peripheral's transmit buffer has been cleared, close the SPI transmission
	// and inform the application that the transmission is over
	if(!pSPIHandle->txLen)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}

} // SPI_TXEInterruptHandle

static void SPI_RXNEInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in SPI_CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;	// Read 16bits of data from the DR
		pSPIHandle->rxLen -= 2;										// Decrement len by 2 (bytes)
		(uint16_t*)pSPIHandle->pRxBuffer++;								// Increment the pointer by 2 bytes
	}
	else
	{
		// 8 BIT DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;				// Read 8bits of data from the DR
		pSPIHandle->rxLen--;											// Decrement len by 1(byte)
		pSPIHandle->pRxBuffer++;										// Increment the pointer by 1 byte
	}

	// If the SPI peripheral's receive buffer has been cleared, close the SPI reception
	// and inform the application that the reception is over
	if(!pSPIHandle->rxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}

} // SPI_RXNEInterruptHandle

static void SPI_OVRERRInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// Clear the OVR flag in SPI TX is not in progress
	if(pSPIHandle->txLen != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	// Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_COMPLETE);

} // SPI_OVRERRInterruptHandle

/**
 * Enable or disable the given IRQ number
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				// Write to ISER0
				*NVIC_ISER0 |= ( 1 << IRQNumber );
			}
			else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
			{
				// Write to ISER1
				*NVIC_ISER1 |= ( 1 << ( IRQNumber % 32 ) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
			{
				// Write to ISER2
				*NVIC_ISER2 |= ( 1 << ( IRQNumber % 64 ) );
			}
		}
		else
		{
			if(IRQNumber <= 31)
			{
				// Write to ICER0
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}
			else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
			{
				// Write to ICER1
				*NVIC_ICER1 |= ( 1 << ( IRQNumber % 32 ) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
			{
				// Write to ICER2
				*NVIC_ICER2 |= ( 1 << ( IRQNumber % 64 ) );
			}
		}
} // SPI_IRQInterruptConfig

/**
 * Set the IRQ priority level for the given IRQ number
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Get IPR register index
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;

	uint8_t bitShiftOffset = ( 8 * iprxSection ) + (8 - NO_PR_BITS_IMPLEMENTED);
	uint32_t *pNvicIprAddr = NVIC_PR_BASEEADDR + ( iprx );	// Since it is uint32_t we move 4 bytes at a time when we increment the address
	*(pNvicIprAddr) |= ( IRQPriority << bitShiftOffset);
} // SPI_IRQPriorityConfig

/**
 * Handle an interrupt for the SPI peripheral
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	// Get the status of the TXE and TXEIE flags
	uint8_t statusFlag = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	uint8_t interruptEnabledFlag = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);
	if(statusFlag && interruptEnabledFlag)
	{
		// Handle TXE interrupt
		SPI_TXEInterruptHandle(pSPIHandle);
	}

	// Get the status of the RXNE and RXNEIE flags
	statusFlag = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	interruptEnabledFlag = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);
	if(statusFlag && interruptEnabledFlag)
	{
		// Handle RXNXE interrupt
		SPI_RXNEInterruptHandle(pSPIHandle);
	}

	// Get the status of the OVR and ERRIE flags
	statusFlag = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	interruptEnabledFlag = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);
	if(statusFlag && interruptEnabledFlag)
	{
		// Handle OVR interrupt
		SPI_OVRERRInterruptHandle(pSPIHandle);
	}

} // SPI_IRQHandling

/**
 * Enable or disable the SPI peripheral
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

} // SPI_PeripheralControl

/**
 * Enable or disable the SPI peripheral internal slave select (SSI bit in CR1)
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

} // SPI_SSIConfig

/**
 * Enable or disable the SPI peripheral slave select output enable (SSOE bit in CR2)
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

} // SPI_SSOEConfig

/**
 * Clear the OVR flag by reading from the DR and SR
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	// Clear the OVR flag by reading from the DR and SR
	uint8_t tmp = 0;
	tmp = pSPIx->DR;
	tmp = pSPIx->SR;
	(void)tmp;

} // SPI_ClearOVRFlag

/**
 * Close SPI peripheral transmission
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	// Clear the TXE Interrupt enable flag to prevent interrupts from being triggered when we have nothing to transmit
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );

	// Reset the TX buffer address, length, and state
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->txState = SPI_READY;
} // SPI_CloseTransmission

/**
 * Close SPI peripheral reception
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// Clear the RXNE Interrupt enable flag to prevent interrupts from being triggered when we have nothing to receive
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );

	// Reset the RX buffer address, length, and state
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_READY;
} // SPI_CloseReception

/**
 * Application callback function
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv)
{
	// This is a weak implementation, it must be overridden by the application to suit its requirements
} // SPI_ApplicationEventCallback
