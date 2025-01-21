/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jan 20, 2025
 *      Author: engineering
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

/**
 * @brief Returns the value of the APB1 bus clock speed in mHz
 * @return	uint32_t	The value of the APB1 bus clock speed in mHz
 */
uint32_t RCC_GetPCLK1Value(void);

/**
 * @brief Returns the value of the APB2 bus clock speed in mHz
 * @return	uint32_t	The value of the APB1 bus clock speed in mHz
 */
uint32_t RCC_GetPCLK2Value(void);


#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
