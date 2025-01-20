/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Jan 19, 2025
 *      Author: engineering
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/**
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t 	USART_Mode;
	uint32_t	USART_Baud;
	uint8_t		USART_NoOfStopBits;
	uint8_t		USART_WordLength;
	uint8_t		USART_ParityControl;
	uint8_t		USART_HWFlowControl;
} USART_Config_t;

/**
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;


#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
