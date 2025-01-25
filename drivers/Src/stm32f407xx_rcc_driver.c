/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jan 20, 2025
 *      Author: engineering
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB_PreScaler[4] = {2, 4, 8, 16};

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
		apb1p = APB_PreScaler[ppre1-4]; // Fetch the prescaler value from the array of possible values
	}

	// Calculate the value of the peripheral clock
	pclk1 = ( ( systemClock / ahb1p ) / apb1p );

	return pclk1;
} // RCC_GetPCLK1Value

/**
 * Returns the value of the APB2 bus clock speed in mHz
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2 = 0;

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
	 * Get the value of the APB2 prescaler by checking the
	 * PPRE2 field in the RCC_CFGR register
	 */
	uint8_t ppre2 = 0;
	ppre2 = ( ( RCC->CFGR >> 13 ) & 0x7 );

	uint16_t apb2p = 0;
	if(ppre2 < 4)
	{
		apb2p = 1;		// No prescaler
	}
	else
	{
		apb2p = APB_PreScaler[ppre2-4]; // Fetch the prescaler value from the array of possible values
	}

	// Calculate the value of the peripheral clock
	pclk2 = ( ( systemClock / ahb1p ) / apb2p );

	return pclk2;
} // RCC_GetPCLK2Value
