/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jan 2, 2025
 *      Author: engineering
 */
#include "stm32f407xx_i2c_driver.h"
#include <string.h>

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

/**
 * Private functions
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t rnwBit);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

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
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
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
 * @param	pI2Cx		I2C peripheral base address
 * @return	void
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
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
 * Returns the value of the APB1 bus clock speed in mHz
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1 = 0;

	/**
	 * Find what clock is being used as the system clock
	 * by checking the SWS field in the RCC_CFGR register
	 */
	uint8_t clkSrc = 0;
	clkSrc = ( ( RCC->CFGR >> 2 ) & 0x3 );

	uint32_t systemClock = 0;

	if(clkSrc == 0)
	{
		// HSI clock source
		systemClock = 16000000; // 16mHz
	}
	else if(clkSrc == 1)
	{
		// HSE clock source
		systemClock = 8000000;	// 8mHz
	}
	else if(clkSrc == 2)
	{
		// PLL clock source

		/**
		 * Unimplemented, we will not be using PLL clock source
		 */

		// systemClock = RCC_GetPLLOutputClock();
	}
	else
	{
		// Invalid clock source
	}

	/**
	 * Get the value of the AHB1 prescalar by reading the HPRE
	 * field in the RCC_CFGR register
	 */
	uint8_t hpre = 0;
	hpre = ( ( RCC->CFGR >> 4 ) & 0xF );

	uint16_t ahb1p = 0;
	if(hpre < 8)
	{
		ahb1p = 1;		// No prescaler
	}
	else
	{
		ahb1p = AHB_PreScaler[hpre-8];	// Fetch the prescaler value from the array of possible values
	}

	/**
	 * Get the value of the APB1 prescaler by checking the
	 * PPRE1 field in the RCC_CFGR register
	 */
	uint8_t ppre1 = 0;
	ppre1 = ( ( RCC->CFGR >> 10 ) & 0x7 );

	uint16_t apb1p = 0;
	if(ppre1 < 4)
	{
		apb1p = 1;		// No prescaler
	}
	else
	{
		apb1p = APB1_PreScaler[ppre1-4]; // Fetch the prescaler value from the array of possible values
	}

	// Calculate the value of the peripheral clock
	pclk1 = ( ( systemClock / ahb1p ) / apb1p );

	return pclk1;
} // RCC_GetPCLK1Value

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
	tmpReg = ( pI2CHandle->I2C_Config.I2C_DeviceAddress << 1 ); 	// Shift one to enforce 7 bit length
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr)
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
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

} // I2C_MasterSendData

/**
 * Receive data
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr)
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
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// Wait until RXNE flag is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

		// Generate the STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Read data into buffer from the DR
		*(pRxBuffer) = pI2CHandle->pI2Cx->DR;
	}
	else if(len > 1)
	{
		// Clear the ADDR flag
		// NOTE: SCL will be stretched (pulled low) until the ADDR flag is cleared
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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

				// Generate the STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

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
 * IRQ configuration and ISR handling
 */
/**
 * @brief	Enable or disable the given IRQ number
 * @param   IRQNumber	IRQ number
 * @param	EnorDi		GPIO pin number
 * @return	void
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief	Set the IRQ priority level for the given IRQ number
 * @param   IRQNumber		IRQ number
 * @param	IRQPriority		Priority level from 0 to 15
 * @return	void
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

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


