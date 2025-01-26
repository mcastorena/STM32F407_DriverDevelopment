/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jan 19, 2025
 *      Author: engineering
 */
#include <string.h>
#include "stm32f407xx_usart_driver.h"

/**
 * Enable/Disable peripheral clock for the given USART peripheral
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else
		{
			// Handle invalid
		}
	}
	else if (EnorDi == DISABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else
		{
			// Handle invalid
		}
	}
} // USART_PeriClockControl

/**
 * Initialize the given USART peripheral
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tmpReg = 0;

	// Enable the clock for the for the USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/**
	 * USART_CR1 configuration
	 */
	// Configure USART TX/RX engines
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tmpReg |= ( 1 << USART_CR1_RE );
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tmpReg |= ( 1 << USART_CR1_TE );
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tmpReg |= (( 1 << USART_CR1_TE ) | ( 1 << USART_CR1_RE ));
	}

	// Configure word length
	tmpReg |= ( pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M );

	// Configure parirty control
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		// Enable parity control
		tmpReg |= ( 1 << USART_CR1_PCE );

		// Parity selection even is 0 by default
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		// Enable parity control
		tmpReg |= ( 1 << USART_CR1_PCE );

		// Parity selection odd
		tmpReg |= ( 1 << USART_CR1_PS );
	}

	// Write to CR1
	pUSARTHandle->pUSARTx->CR1 = tmpReg;

	/**
	 * USART CR3 configuration
	 */
	tmpReg = 0;

	// Configure hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tmpReg |= ( 1 << USART_CR3_CTSE );
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tmpReg |= ( 1 << USART_CR3_RTSE );
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tmpReg |= (( 1 << USART_CR3_CTSE ) | ( 1 << USART_CR3_RTSE ));
	}
	// Write to CR3
	pUSARTHandle->pUSARTx->CR3 = tmpReg;

	/**
	 * USART BRR configuration
	 */
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
} // USART_Init

/**
 * De-initialize the given USART peripheral
 */
void USART_DeInit(USART_Handle_t *pUSARTHandle)
{

} //USART_DeInit

/**
 * Initializes the GPIO pins for USART usage
 */
void USART_GPIOInit(GPIO_RegDef_t *pGPIOx, uint8_t AFMode, uint8_t RxPin, uint8_t TxPin)
{
	// Create and initialize GPIO Handle
	GPIO_Handle_t I2CPins;
	memset(&I2CPins, 0, sizeof(I2CPins));
	I2CPins.pGPIOx = pGPIOx;

	// Set general pin configuration
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = AFMode;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	// Push-pull output type
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;	// Internal pull-up resistors
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// Init SCLK
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = RxPin;
	GPIO_Init(&I2CPins);

	// Init MOSI
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = TxPin;
	GPIO_Init(&I2CPins);
} // I2C_GPIOInit

/**
 * Send data
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pData;

	// Loop until len number of bytes are transferred
	for(uint32_t i = 0; i < len; i++)
	{
		// Wait until the TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		// Check the word length configuration
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// Load the DR with 2bytes and mask all bits other than the first 9bits
			pData = (uint16_t *)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

			// Check parity control configuration
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// 9bits of data were sent, increment buffer address by 2bytes
				pTxBuffer += 2;
			}
			else
			{
				// 8bits of data sent, increment buffer address by one byte
				pTxBuffer++;
			}
		}
		else
		{
			// 8bit transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			// Increment buffer address one byte
			pTxBuffer++;
		}
	}

	// Wait until the TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));

} // USART_SendData

/**
 * Receive data
 */
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len)
{

} // USART_ReceiveData

/**
 * Interrupt based send data
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len)
{
	return 1;
} // USART_SendDataIT

/**
 * Interrupt based receive data
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len)
{
	return 1;
} // USART_ReceiveDataIT

/**
 * Enable or disable the USART peripheral
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= ( 1 << USART_CR1_UE );
	}
	else
	{
		pUSARTx->CR1 &= ~( 1 << USART_CR1_UE );
	}
} // USART_PeripheralControl

/**
 * Enable or disable the given IRQ number
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
} // USART_IRQInterruptConfig

/**
 * Set the IRQ priority level for the given IRQ number
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Get IPR register index
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;

	uint8_t bitShiftOffset = ( 8 * iprxSection ) + (8 - NO_PR_BITS_IMPLEMENTED);
	uint32_t *pNvicIprAddr = NVIC_PR_BASEEADDR + ( iprx );	// Since it is uint32_t we move 4 bytes at a time when we increment the address
	*(pNvicIprAddr) |= ( IRQPriority << bitShiftOffset);
} // USART_IRQPriorityConfig

/**
 * Retrieves flag status from the USART_SR register
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName)
{
	uint8_t flagStatus = FLAG_RESET;

	if( pUSARTx->SR & flagName)
	{
		flagStatus = FLAG_SET;
	}

	return flagStatus;
} // USART_GetFlagStatus

/**
 * Resets flag status in the USART_SR register
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t flagName)
{
	pUSARTx->SR &= ~(flagName);
} // USART_ClearFlag

/**
 * Calculates the USARTDIV value needed to achieve the desired baud rate and write to the BRR register
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t PCLKx = 0;		// APB Clock value
	uint32_t USARTDIV = 0;

	// Mantissa and Fractional values
	uint32_t mantissaValue, fractionalValue;

	uint32_t tmpReg = 0;

	// Get the value of the APB bus clock that the USART peripheral is attached to
	if((pUSARTx == USART1))
	{
		// USART1 & 6 are hanging on the APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration in the CR1 register
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8 ))
	{
		// Oversampling by 8
		USARTDIV = ((25 * PCLKx) / (2 * BaudRate));
	}
	else
	{
		// Oversampling by 16
		USARTDIV = ((25 * PCLKx) / (4 * BaudRate));
	}

	// Calculate Mantissa value
	mantissaValue = USARTDIV/100;

	// Place Mantissa value in the appropriate bit position
	tmpReg |= mantissaValue << 4;

	// Calculate Fractional value
	fractionalValue = (USARTDIV - (mantissaValue * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		fractionalValue = ((( fractionalValue * 8)+ 50) / 100)& ((uint8_t)0x07);
	}
	else
	{
		//over sampling by 16
		fractionalValue = ((( fractionalValue * 16)+ 50) / 100) & ((uint8_t)0x0F);
   }
	// Write the fractional value
	tmpReg |= fractionalValue;

	// Write to BRR register
	pUSARTx->BRR = tmpReg;

} // USART_SetBaudRate

