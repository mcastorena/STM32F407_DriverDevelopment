/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 16, 2024
 *      Author: engineering
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PIN_>*/
	uint8_t GPIO_PinOPType;			/*!< possible values from @GPIO_PIN_>*/
	uint8_t GPIO_PinAltFunMode;		/*!< possible values from @GPIO_PIN_>*/
}GPIO_PinConfig_t;

/**
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t 		*pGPIOx;			// This holds the base address of the GPIO port which the pin belongs to
	GPIO_PinConfig_t	GPIO_PinConfig;		// This holds GPIO pin configuration settings
}GPIO_Handle_t;

/**
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/**
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4	// Input, interrupt on falling edge trigger
#define GPIO_MODE_IT_RT		5	// Input, interrupt on rising edge trigger
#define GPIO_MODE_IT_RFT	6	// Input, interrupt on rising edge & falling edge trigger

/**
 * @GPIO_PIN
 * GPIO pin output types
 */
#define GPIO_OP_TYPE_PP 	0	// push-pull
#define GPIO_OP_TYPE_OD		1	// open drain

/**
 * GPIO pin output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/**
 * GPIO pull-up/pull-down configurations
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*********************** APIs supported by this driver *********************************/


/**
 * Peripheral Clock setup
 */

/**
 * @brief   Enable/Disable peripheral clock for the given GPIO port
 * @param   pGPIOx    GPIO Peripheral base address
 * @param   EnorDi    ENABLE or DISABLE macro
 * @return  void
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * Init and De-init
 */

/**
 * @brief   Initialize the given GPIO port
 * @param   pGPIOHandle    	GPIO Handle
 * @return  void
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief   De-initialize the given GPIO port
 * @param   pGPIOx    GPIO Peripheral base address
 * @return  void
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/**
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
