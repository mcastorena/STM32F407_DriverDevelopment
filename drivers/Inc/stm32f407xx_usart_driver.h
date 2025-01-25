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

/*
 *@USART_Mode
 */
#define USART_MODE_ONLY_TX 			0
#define USART_MODE_ONLY_RX 			1
#define USART_MODE_TXRX  			2

/*
 *@USART_Baud
 */
#define USART_STD_BAUD_1200			1200
#define USART_STD_BAUD_2400			2400
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200 		19200
#define USART_STD_BAUD_38400 		38400
#define USART_STD_BAUD_57600 		57600
#define USART_STD_BAUD_115200 		115200
#define USART_STD_BAUD_230400 		230400
#define USART_STD_BAUD_460800 		460800
#define USART_STD_BAUD_921600 		921600
#define USART_STD_BAUD_2M 			2000000
#define SUART_STD_BAUD_3M 			3000000

/*
 *@USART_ParityControl
 */
#define USART_PARITY_EN_ODD   		2
#define USART_PARITY_EN_EVEN  		1
#define USART_PARITY_DISABLE   		0

/*
 *@USART_WordLength
 */
#define USART_WORDLEN_8BITS  		0
#define USART_WORDLEN_9BITS  		1

/*
 *@USART_NoOfStopBits
 */
#define USART_STOPBITS_1     		0
#define USART_STOPBITS_0_5   		1
#define USART_STOPBITS_2     		2
#define USART_STOPBITS_1_5   		3


/*
 *@USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/**
 * USART flag status macros
 */
#define USART_FLAG_PE				( 1 << USART_SR_PE )
#define USART_FLAG_FE				( 1 << USART_SR_FE )
#define USART_FLAG_NF				( 1 << USART_SR_NF )
#define USART_FLAG_ORE				( 1 << USART_SR_ORE )
#define USART_FLAG_IDLE				( 1 << USART_SR_IDLE )
#define USART_FLAG_RXNE				( 1 << USART_SR_RXNE )
#define USART_FLAG_TC				( 1 << USART_SR_TC )
#define USART_FLAG_TXE				( 1 << USART_SR_TXE )
#define USART_FLAG_LBD				( 1 << USART_SR_LBD )
#define USART_FLAG_CTS				( 1 << USART_SR_CTS )

/*
 * USART application states
 */
#define USART_READY 				0
#define USART_BUSY_IN_RX 			1
#define USART_BUSY_IN_TX 			2

/**
 * @brief   Enable/Disable peripheral clock for the given USART peripheral
 * @param   pUSARTx   USART Peripheral base address
 * @param   EnorDi    ENABLE or DISABLE macro
 * @return  void
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/**
 * @brief 	Enable or disable the USART peripheral
 * @note	You must configure and initialize the USART peripheral before enabling it
 * @param	pUSARTx		USART peripheral base address
 * @param	EnorDi		ENABLE or DISABLE macro
 * @return	void
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
/**
 * @brief   Initialize the given USART peripheral
 * @param   pUSARTHandle    USART Handle
 * @return  void
 */
void USART_Init(USART_Handle_t *pUSARTHandle);

/**
 * @brief   De-initialize the given USART peripheral
 * @param   pUSARTHandle    USART Handle
 * @return  void
 */
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/**
 * @brief   Initializes the GPIO pins for USART usage
 * @param   pGPIOx		GPIO Peripheral base address
 * @param	AFMode		Alternate Function mode
 * @param	RxPin		Pin number for USART RW
 * @param	TxPin		Pin number for USART TX
 * @return  void
 */
void USART_GPIOInit(GPIO_RegDef_t *pGPIOx, uint8_t AFMode, uint8_t RxPin, uint8_t TxPin);

/*
 * Data Send and Receive
 */
/**
 * @brief	Send data
 * @param	pUSARTHandle 	USART Peripheral Handle
 * @param	pTxBuffer		Pointer to the transmit buffer
 * @param	len				Size of the data we want to transmit
 * @return	void
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief	Receive data
 * @param	pUSARTHandle 	USART Peripheral Handle
 * @param	pRxBuffer		Pointer to the receive buffer
 * @param	len				Size of the data we want to receive
 * @return	void
 */
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief	Interrupt based send data
 * @param	pUSARTHandle 	USART Peripheral Handle
 * @param	pTxBuffer		Pointer to the transmit buffer
 * @param	len				Size of the data we want to transmit
 * @return	void
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief	Interrupt based receive data
 * @param	pUSARTHandle 	USART Peripheral Handle
 * @param	pRxBuffer		Pointer to the receive buffer
 * @param	len				Size of the data we want to receive
 * @return	void
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len);

/**
 * IRQ configuration and ISR handling
 */
/**
 * @brief	Enable or disable the given IRQ number
 * @param   IRQNumber	IRQ number
 * @param	EnorDi		GPIO pin number
 * @return	void
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief	Set the IRQ priority level for the given IRQ number
 * @param   IRQNumber		IRQ number
 * @param	IRQPriority		Priority level from 0 to 15
 * @return	void
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other Peripheral Control APIs
 */
/**
 * @brief   Retrieves flag status from the USART_SR register
 * @param   pUSARTx		USART Peripheral base address
 * @param	flagName 	Flag to check status
 * @return  uint8_t	Flag set or reset
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName);

/**
 * @brief   Resets flag status in the USART_SR register
 * @param   pUSARTx		USART Peripheral base address
 * @param  	flagName	Flag to reset
 * @return	void
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t flagName);

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/**
 * @brief   Calculates the USARTDIV value needed to achieve the desired
 * 			baud rate and write to the BRR register
 * @param   pUSARTx		USART Peripheral base address
 * @param  	BaudRate	Desired baud rate
 * @return	void
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);



#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
