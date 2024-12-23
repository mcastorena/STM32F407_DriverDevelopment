/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Dec 16, 2024
 *      Author: engineering
 */


#include "stm32f407xx_gpio_driver.h"


/**
 * Peripheral Clock setup
 */

/**
 * Enable/Disable peripheral clock for the given GPIO port
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
        else
        {
            // Handle invalid GPIO port
        }
    }
    else if (EnorDi == DISABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
        else
        {
            // Handle invalid GPIO port
        }
    }
}
 // GPIO_PeriClockControl

/**
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t tmp = 0;	// tmp register

	// Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)	// Non-interrupt mode
	{
		// Clear the pin's bitfields
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

		// Left shift to the pin's bit position in the GPIO pin mode register
		tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER |= tmp;
	}
	else	// Interrupt mode
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the Falling Trigger Selection Register (FTSR)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

			// Ensure rising edge trigger selection is disabled by clearing the bit field
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the Rising Trigger Selection Register (RTSR)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

			// Ensure falling edge trigger selection is disabled by clearing the bit field
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure both the Falling and Rising Trigger Selection Register (FTSR)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		// Configure the GPIO port selection register in SYSCFG_EXITCR
		SYSCFG_PCLK_EN();	// First enable the SYSCFG peripheral clock (APB2)

		// Enable control of the corresponding EXTI line for this pin
		uint8_t controlRegisterIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t bitShiftOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[controlRegisterIndex] = ( portCode << (4 * bitShiftOffset) );

		// Enable the EXTI interrupt delivery using the Interrupt Mask register (IMR)
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}
	tmp = 0;

	// Configure the speed
	// Clear the pin's bitfields
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR |= tmp;

	tmp = 0;

	// Configure pull-up/pull-down settings
	// Clear the pin's bitfields
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR |= tmp;

	tmp = 0;

	// Configure the output type
	// Clear the pin's bitfield
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER |= tmp;

	tmp = 0;

	// Configure the alternate functionality if required
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Determine if the pin configuration belongs to the high or low alternate function register
		uint8_t regPosition = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= 8 );
		uint8_t bitShiftOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		// Clear the pin's bitfields
		pGPIOHandle->pGPIOx->ARF[regPosition] &= ~( 0xF << ( 4 * bitShiftOffset ) );

		tmp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * bitShiftOffset ) );
		pGPIOHandle->pGPIOx->ARF[regPosition] |= tmp;
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else
	{
		// Handle invalid GPIO port
	}
} // GPIO_DeInit

/**
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value = 0;

	// Shift the contents of the input data register to move the pin's input data to the 0th bit and mask all other bits
	value = (uint8_t) ( (pGPIOx->IDR >> pinNumber) & 0x00000001 );

	return value;
} // GPIO_ReadFromInputPin

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = 0;
	value = (uint16_t)pGPIOx->IDR;
	return value;
} // GPIO_ReadFromInputPort

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
} // GPIO_WriteToOutputPin

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
} // GPIO_WriteToOutputPort

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= ( 1 << pinNumber ) ;
} // GPIO_ToggleOutputPin

/**
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Get IPR register index
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;

	uint8_t bitShiftOffset = ( 8 * iprxSection ) + (8 - NO_PR_BITS_IMPLEMENTED);
	uint32_t *pNvicIprAddr = NVIC_PR_BASEEADDR + ( iprx );	// Since it is uint32_t we move 4 bytes at a time when we increment the address
	*(pNvicIprAddr) |= ( IRQPriority << bitShiftOffset);
}

void GPIO_IRQHandling(uint8_t pinNumber)
{
	// Clear the EXTI Pending Register corresponding to the pin number
	if(EXTI->PR & (1 << pinNumber))
	{
		// Clear the pending register bit by writing 1
		EXTI->PR |= ( 1 << pinNumber );
	}
}
