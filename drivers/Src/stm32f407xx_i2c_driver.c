/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jan 2, 2025
 *      Author: engineering
 */
#include "stm32f407xx_i2c_driver.h"
#include <string.h>

/**
 * Private functions
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t rnwBit);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/**
 * @brief	Helper function used to generate the I2C START condition
 * @param	pI2Cx		I2C peripheral base address
 * @return	void
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START );
}  // I2C_GenerateStartCondition

/**
 * @brief	Helper function used to generate the I2C STOP condition
 * @param	pI2Cx		I2C peripheral base address
 * @return	void
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP );
}  // I2C_GenerateStopCondition

/**
 * @brief	Helper function used to execute the I2C address phase
 * @param	pI2Cx		I2C peripheral base address
 * @param	slaveAddr	I2C slave address
 * @param	rnwBit		R/nW bit of the address phase, read or write
 * @return	void
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t rnwBit)
{
	uint8_t addressData = ( slaveAddr << 1 );	// Shift to make room for the R/nW bit at the 0th bit position

	if(rnwBit == I2C_WRITE_BIT)
	{
		addressData &= ~(0x1);						// Clear the 0th bit for W (0)
	}
	else
	{
		addressData |= (0x1);						// Set the 0th bit for R (1)
	}

	pI2Cx->DR = addressData;					// Write to data register
} // I2C_ExecuteAddressPhase

/**
 * @brief	Helper function used to clear the I2C ADDR flag
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @return	void
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	// Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ))
	{
		// Device is in Master mode
		if(pI2CHandle->txRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->rxSize == 1)
			{
				// Disable the ACK
				I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
			}
		}
	}
	else
	{
		// Device is in Slave mode
	}
	// Clear the ADDR flag by reading SR1 and SR2
	uint32_t dummyRead = pI2CHandle->pI2Cx->SR1;
	dummyRead = pI2CHandle->pI2Cx->SR2;
	(void)dummyRead;
} // I2C_ClearADDRFlag

/**
 * Enable/Disable peripheral clock for the given I2C peripheral
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
		else
		{
			// Handle invalid
		}
	}
	else if (EnorDi == DISABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
		else
		{
			// Handle invalid
		}
	}
} // I2C_PeriClockControl

/**
 * Initialize the given I2C peripheral
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tmpReg = 0;

	// Enable the peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Configure the FREQ field of CR2
	tmpReg = 0;
	tmpReg = ( RCC_GetPCLK1Value()/1000000U );
	pI2CHandle->pI2Cx->CR2 = ( tmpReg & 0x3F );		// Mask all but the first 5 bits to set the FREQ field

	// Configure the device address
	tmpReg = 0;
	tmpReg |= ( pI2CHandle->I2C_Config.I2C_DeviceAddress << 1 ); 	// Shift one to enforce 7 bit length
	tmpReg |= ( 1 << 14 );		// Bit 14 must be kept at 1 by software, refer to reference manual section 27.6.3 I2C Own address register 1 (I2C_OAR1)
	pI2CHandle->pI2Cx->OAR1 = tmpReg;

	// CCR Calculations
	uint16_t ccrValue = 0;
	tmpReg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		ccrValue = ( RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tmpReg |= ( ccrValue & 0xFFF );	// Mask out all bits besides the first 12
	}
	else
	{
		// Fast mode
		tmpReg |= ( 1 << I2C_CCR_FS ); 	// Set the F/S field to fast mode
		tmpReg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY );	// Configure the duty cycle

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccrValue = ( RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		else
		{
			ccrValue = ( RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tmpReg |= ( ccrValue & 0xFFF );	// Mask out all bits besides the first 12
	}
	pI2CHandle->pI2Cx->CCR = tmpReg;

	// Calculate TRISE
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		tmpReg = ( RCC_GetPCLK1Value() / 1000000U ) + 1;
	}
	else
	{
		// Fast mode
		tmpReg = ( ( RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tmpReg & 0x3F); // Mask all bits but the first 5

} // I2C_Init

/**
 * @brief   De-initialize the given I2C peripheral
 * @param   pI2Cx    I2C Peripheral base address
 * @return  void
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/**
 * Initializes the GPIO pins for I2C usage
 */
void I2C_GPIOInit(GPIO_RegDef_t *pGPIOx, uint8_t AFMode, uint8_t SCLPin, uint8_t SDAPin)
{
	// Create and initialize GPIO Handle
	GPIO_Handle_t I2CPins;
	memset(&I2CPins, 0, sizeof(I2CPins));
	I2CPins.pGPIOx = pGPIOx;

	// Set general pin configuration
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = AFMode;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;	// Open drain output type
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;	// We will use an external pull-up resistors
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// Init SCLK
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = SCLPin;
	GPIO_Init(&I2CPins);

	// Init MOSI
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = SDAPin;
	GPIO_Init(&I2CPins);
} // I2C_GPIOInit

/**
 * Retrieves flag status from the I2C_SR1 register
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flagName)
{
	uint8_t flagStatus = FLAG_RESET;

	if( pI2Cx->SR1 & flagName)
	{
		flagStatus = FLAG_SET;
	}

	return flagStatus;
} // I2C_GetFlagStatus

/**
 * Data send and receive
 */
/**
 * Send data
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr)
{
	// Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Confirm that the START generation is complete by checking the SB field in the SR1 register
	// NOTE: SCL will be stretched (pulled low) until the SB flag is cleared
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));	// Wait until the SB flag is set

	// Send the Slave Address with the R/nW bit set to W (0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, I2C_WRITE_BIT);

	// Confirm the address phase is completed by checking the ADDR flag in the SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG)); // Wait until the ADDR flag is set

	// Clear the ADDR flag
	// NOTE: SCL will be stretched (pulled low) until the ADDR flag is cleared
	I2C_ClearADDRFlag(pI2CHandle);

	// Send data until len = 0
	while(len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));		// Wait until the TXE flag is set
		pI2CHandle->pI2Cx->DR = *(pTxBuffer);							// Load byte until the data register
		pTxBuffer++;													// Advance buffer pointer by 1 byte
		len--;															// Decrement length
	}

	// After sending all of the data, wait until TXE = 1 and BTF = 1 before generating the STOP condition
	// NOTE: When TXE = 1, BTF = 1, this means both the SR and DR are empty and next transmission should begin
	// When BTF = 1, SCL will be stretched (pulled low)
	while( (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)) && (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG)) );

	if(Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

} // I2C_MasterSendData

/**
 * Receive data
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr)
{
	// Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Confirm that the START generation is complete by checking the SB field in the SR1 register
	// NOTE: SCL will be stretched (pulled low) until the SB flag is cleared
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));	// Wait until the SB flag is set

	// Send the Slave Address with the R/nW bit set to W (0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, I2C_READ_BIT);

	// Confirm the address phase is completed by checking the ADDR flag in the SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG)); // Wait until the ADDR flag is set

	// Reading only one byte from the slave
	if(len == 1 )
	{
		// Disable ACKing
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// Clear the ADDR flag
		// NOTE: SCL will be stretched (pulled low) until the ADDR flag is cleared
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait until RXNE flag is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

		// Generate the STOP condition if needed
		if(Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Read data into buffer from the DR
		*(pRxBuffer) = pI2CHandle->pI2Cx->DR;
	}
	else if(len > 1)
	{
		// Clear the ADDR flag
		// NOTE: SCL will be stretched (pulled low) until the ADDR flag is cleared
		I2C_ClearADDRFlag(pI2CHandle);

		// Read the data until len = 0
		for(uint32_t i = len; i > 0; i--)
		{
			// Wait until RXNE flag is set
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

			// If the last two bytes are remaining
			if(i == 2)
			{
				// Disable ACKing
				I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Generate the STOP condition if needed
				if(Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				/**
				 * The Master will send NACK and generate the STOP condition
				 * after the Read on this iteration of the loop then
				 * read in the final byte on the next and final iteration
				 */
			}

			// Read data into buffer from the DR
			*(pRxBuffer) = pI2CHandle->pI2Cx->DR;

			// Advance the pointer and decrement len
			pRxBuffer++;
			len--;
		}
	}

	// Re-Enable ACKing if needed
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

} // I2C_MasterReceiveData

/**
 * Interrupt driven send data
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr)
{
	uint8_t busyState = pI2CHandle->txRxState;

	if((busyState != I2C_BUSY_IN_RX) && (busyState != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->txLen = len;
		pI2CHandle->txRxState = I2C_BUSY_IN_TX;
		pI2CHandle->devAddr = slaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

		// Enable the ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );

		// Enable the ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}

	return busyState;
} // I2C_MasterSendDataIT

/**
 * Interrupt driven receive data
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr)
{
	uint8_t busyState = pI2CHandle->txRxState;

	if((busyState != I2C_BUSY_IN_RX) && (busyState != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->rxLen = len;
		pI2CHandle->rxSize = len;
		pI2CHandle->txRxState = I2C_BUSY_IN_RX;
		pI2CHandle->devAddr = slaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

		// Enable the ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );

		// Enable the ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}

	return busyState;
} // I2C_MasterReceiveDataIT

/**
 * Send data in Slave mode
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	// Load the byte of data into the I2C peripheral Data Register
	pI2Cx->DR = data;
} // I2C_SlaveSendData

/**
 * Receive data in Slave mode
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	// Return the byte of data in the I2C peripheral Data Register
	return (uint8_t)pI2Cx->DR;
} // I2C_SlaveReceiveData

/**
 * IRQ configuration and ISR handling
 */

/**
 * Enable or disable the given IRQ number
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
} // I2C_IRQInterruptConfig

/**
 * Set the IRQ priority level for the given IRQ number
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Get IPR register index
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;

	uint8_t bitShiftOffset = ( 8 * iprxSection ) + (8 - NO_PR_BITS_IMPLEMENTED);
	uint32_t *pNvicIprAddr = NVIC_PR_BASEEADDR + ( iprx );	// Since it is uint32_t we move 4 bytes at a time when we increment the address
	*(pNvicIprAddr) |= ( IRQPriority << bitShiftOffset);
} // I2C_IRQPriorityConfig


/**
 * Handles interrupts generated on the I2C_EV line
 */
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle)
{
	uint32_t ITEVTENSet, ITBUFENSet, statusFlag;

	ITEVTENSet = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );	// Get the status of the ITEVTEN in CR2
	ITBUFENSet = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );	// Get the status of the ITBUFEN in CR2

	statusFlag = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG);	// Check if the SB flag is set in SR1S
	if(ITEVTENSet && statusFlag)
	{
		// Handle interrupt generated by SB event
		// NOTE: SB flag is only applicable in Master mode, in Slave mode the flag will always be 0
		// Slave devices will not generate START condition
		// After generating the START condition we will then commence the Address Phase
		uint8_t rnwBit = (pI2CHandle->txRxState == I2C_BUSY_IN_TX) ? I2C_WRITE_BIT : I2C_READ_BIT;	// Check the application state to set the R/nW bit
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->devAddr, rnwBit);
	}

	statusFlag = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG);	// Check if the ADDR flag is set in SR1
	if(ITEVTENSet && statusFlag)
	{
		// Handle interrupt generated by ADDR event
		// NOTE: In master mode: address is sent
		//		 In slave mode: address is matched with its own address
		I2C_ClearADDRFlag(pI2CHandle);		// Clear the ADDR flag
	}

	statusFlag = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG);	// Check if the BTF flag is set in SR1
	if(ITEVTENSet && statusFlag)
	{
		// Handle interrupt generated by BTF (Byte Transfer Finished) event
		if(pI2CHandle->txRxState == I2C_BUSY_IN_TX)
		{
			// Close the communication if all bytes have been transmitted
			if(pI2CHandle->txLen == 0)
			{
				// Check to see if the TXE flag is also set
				if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG))
				{
					// Generate the STOP condition if needed
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// Reset all the I2C data transmission member elements of the Handle struct
					I2C_CloseDataTransmission(pI2CHandle);

					// Notify the application that the transmission is complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
				}
			}
		}
		else if(pI2CHandle->txRxState == I2C_BUSY_IN_RX)
		{
			// Check to see if the RXNE flag is also set
			if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG))
			{
				// Reading only one byte from the slave
				if(pI2CHandle->rxSize == 1)
				{
					// Read one byte of data into the RX buffer from the DR
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
					pI2CHandle->rxLen--;		// Decrement len
				}
				else if(pI2CHandle->rxLen > 1)
				{
					// If the last two bytes are remaining
					if(pI2CHandle->rxLen == 2)
					{
						// Disable ACKing
						I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
					}

					// Read data into buffer from the DR
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

					// Advance the pointer and decrement len
					pI2CHandle->pRxBuffer++;
					pI2CHandle->rxLen--;
				}
				else if(pI2CHandle->rxLen == 0)
				{
					// Generate the STOP condition if needed
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// Close the I2C data reception
					I2C_CloseDataReception(pI2CHandle);

					// Notify the application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
				}
			}

			// Re-Enable ACKing if needed
			if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
			{
				I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
			}
		}
	}

	statusFlag = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STOPF_FLAG);	// Check if the STOPF flag is set in SR1
	if(ITEVTENSet && statusFlag)
	{
		// Handle interrupt generated by STOPF event
		// Note: Slave mode only, STOPF will not be set in Master mode
		// Set by hardware when a Stop condition is detected on the bus by the slave after an ACK

		// Clear the STOPF flag by reading SR1 (completed above) then writing to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;	// Dummy write will not affect CR1's contents

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	statusFlag = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG);	// Check if the TXE flag is set in SR1
	if(ITEVTENSet && ITBUFENSet && statusFlag)
	{
		// Handle interrupt generated by TXE event
		// Transmit data if the device is in master mode and the application state is BUSY_IN_TX
		if((pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) && (pI2CHandle->txRxState == I2C_BUSY_IN_TX))
		{
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
		else
		{
			// Slave Mode
			// Check the TRA bitfield in SR2 register to see if we are in Transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA ))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}

	statusFlag = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG);	// Check if the RXNE flag is set in SR1
	if(ITEVTENSet && ITBUFENSet && statusFlag)
	{
		// Handle interrupt generated by RXNE event
		// Receive data if the device is in master mode and the application state is BUSY_IN_RX
		if((pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) && (pI2CHandle->txRxState == I2C_BUSY_IN_RX))
		{
			I2C_MasterHandleRXNEInterrupt(pI2CHandle);
		}
		else
		{
			// Slave Mode
			// Check the TRA bitfield in SR2 register to see if we are in Transmitter mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA )))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
} // I2C_EV_IRQHandler

/**
 * Handles interrupts generated on the I2C_ER line
 */
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle)
{
	uint32_t ITERRENSet = ( pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITERREN) );

	// Handle error interrupts if the ITERREN control bit is set in the CR2 register
	if(ITERRENSet)
	{
		// Check for Bus Error
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BERR) == FLAG_SET)
		{
			// Clear the Bus Error Flag by writing 0 to the SR1 BERR bitfield
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR );

			// Notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

		// Check for Arbitration Loss Error
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ARLO) == FLAG_SET)
		{
			// Clear the ARLO Error Flag by writing 0 to the SR1 ARLO bitfield
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO );

			// Notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
		}

		// Check for ACK Error
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_AF) == FLAG_SET)
		{
			// Clear the AF Flag by writing 0 to the SR1 AF bitfield
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF );

			// Notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF );
		}

		// Check for Overrun/Underrun Error
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_OVR) == FLAG_SET)
		{
			// Clear the OVR Flag by writing 0 to the SR1 OVR bitfield
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR );

			// Notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR );
		}

		// Check for Time Out Error
		if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TIMEOUT) == FLAG_SET)
		{
			// Clear the TIMEOUT Flag by writing 0 to the SR1 TIMEOUT bitfield
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT );

			// Notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}
	}
} // I2C_ER_IRQHandler

/**
 * Handles TXE interrupts when the I2C Peripheral is in Master mode
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->txLen > 0)
	{
		// Load data into the DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		// Decrement txLen
		pI2CHandle->txLen--;

		// Advance txBuffer address pointer
		pI2CHandle->pTxBuffer++;
	}
} // I2C_MasterHandleTXEInterrupt

/**
 * Handles RXNE interrupts when the I2C Peripheral is in Master mode
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->rxSize == 1)
	{
		// Read one byte into the RX Buffer and decrement the length
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->rxLen--;
	}
	else if(pI2CHandle->rxSize > 1)
	{
		if(pI2CHandle->rxLen == 2)
		{
			// Disable ACKing
			I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}

		// Read data into the RX Buffer
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->rxLen--;
	}

	// Close the I2C data reception if len is 0 and notify the application
	if(pI2CHandle->rxLen == 0)
	{
		// Generate the STOP condition if needed
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Close I2C data reception
		I2C_CloseDataReception(pI2CHandle);

		// Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
	}
} // I2C_MasterHandleRXNEInterrupt

void I2C_CloseDataReception(I2C_Handle_t *pI2CHandle)
{
	// Disable the ITBUFEN Control Bit in CR2
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	// Disable the ITEVTEN Control Bit in CR2
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	// Reset I2C Handle RX data members
	pI2CHandle->txRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->rxLen = 0;
	pI2CHandle->rxSize = 0;

	// Enable ACK if configured
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
} // I2C_CloseDataReception

void I2C_CloseDataTransmission(I2C_Handle_t *pI2CHandle)
{
	// Disable the ITBUFEN Control Bit in CR2
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	// Disable the ITEVTEN Control Bit in CR2
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	// Reset I2C Handle RX data members
	pI2CHandle->txRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->txLen = 0;

	// Enable ACK if configured
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
} // I2C_CloseDataTransmission

/**
 * Other Peripheral Control APIs
 */

/**
 * Enable or disable the I2C peripheral
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_PE );
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE );
	}
} // I2C_PeripheralControl

/**
 * Enable or disable ACKing for the I2C peripheral
 */
void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK );
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK );
	}
} // I2C_ACKControl

/**
 * Enable or disable interrupt callback events for the I2C peripheral
 */
void I2C_CallbackEventsControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		// Enable the ITBUFEN control bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

		// Enable the ITEVTEN control bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );

		// Enable the ITERREN control bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}
	else
	{
		// Disable the ITBUFEN control bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

		// Disable the ITEVTEN control bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

		// Disable the ITERREN control bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN );
	}
}

/**
 * Application call-back
 */
/**
 * @brief	Application callback function
 * @note	This is a weak implementation, it must be overridden by
 * 			the application to suit its requirements
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @param	appEv		Application event macro
 * @return	void
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv);


