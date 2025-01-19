/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Jan 2, 2025
 *      Author: engineering
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/**
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

/**
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t			*pTxBuffer;		// Stores the application TX Buffer address
	uint8_t			*pRxBuffer;		// Stores the application RX Buffer address
	uint32_t		txLen;			// Length of TX Buffer
	uint32_t		rxLen;			// Length of RX Buffer
	uint8_t			txRxState;		// Stores the communication state
	uint8_t			devAddr;		// Stores the slave/device address
	uint32_t		rxSize;			// Stores total RX buffer size
	uint8_t			Sr;				// Stores the Repeated Start value
}I2C_Handle_t;

/**
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000		// Standard mode, 100KHz
#define I2C_SCL_SPEED_FM2K		200000		// Fast mode, 200KHz
#define I2C_SCL_SPEED_FM4K		400000		// Fast mode, 400KHz

/**
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/**
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/**
 * I2C Flag Status macros
 */
#define I2C_TXE_FLAG			( 1 << I2C_SR1_TXE )
#define I2C_RXNE_FLAG			( 1 << I2C_SR1_RXNE )
#define I2C_SB_FLAG				( 1 << I2C_SR1_SB )
#define I2C_ADDR_FLAG			( 1 << I2C_SR1_ADDR )
#define I2C_BTF_FLAG			( 1 << I2C_SR1_BTF )
#define I2C_STOPF_FLAG			( 1 << I2C_SR1_STOPF )
#define I2C_BERR_FLAG			( 1 << I2C_SR1_BERR )
#define I2C_ARLO_FLAG			( 1 << I2C_SR1_ARLO )
#define I2C_AF_FLAG				( 1 << I2C_SR1_AF )
#define I2C_OVR_FLAG			( 1 << I2C_SR1_OVR )
#define I2C_TIMEOUT_FLAG		( 1 << I2C_SR1_TIMEOUT )

/**
 * I2C R/nW bit macros
 */
#define I2C_WRITE_BIT			0
#define I2C_READ_BIT			1

/**
 * I2C Sr (Repeated Start) Condition Flags
 */
#define I2C_DISABLE_SR			RESET
#define I2C_ENABLE_SR			SET

/**
 * I2C Application States
 */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

/**
 * I2C Application Callback Events
 */
#define I2C_EV_TX_COMPLETE		0
#define I2C_EV_RX_COMPLETE		1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RCV			9

/*********************** APIs supported by this driver *********************************/

/**
 * Peripheral Clock setup
 */

/**
 * @brief   Enable/Disable peripheral clock for the given I2C peripheral
 * @param   pGPIOx    I2C Peripheral base address
 * @param   EnorDi    ENABLE or DISABLE macro
 * @return  void
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * @brief Returns the value of the APB1 bus clock speed in mHz
 * @return	uint32_t	The value of the APB1 bus clock speed in mHz
 */
uint32_t RCC_GetPCLK1Value(void);

/**
 * Init and De-init
 */
/**
 * @brief   Initialize the given I2C peripheral
 * @param   pI2CHandle    I2C Handle
 * @return  void
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);

/**
 * @brief   De-initialize the given I2C peripheral
 * @param   pI2Cx    I2C Peripheral base address
 * @return  void
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/**
 * @brief   Initializes the GPIO pins for I2C usage
 * @param   pGPIOx		GPIO Peripheral base address
 * @param	AFMode		Alternate Function mode
 * @param	SCLPin		Pin number for I2C SCL
 * @param	SDAPin		Pin number for I2C SDA
 * @return  void
 */
void I2C_GPIOInit(GPIO_RegDef_t *pGPIOx, uint8_t AFMode, uint8_t SCLPin, uint8_t SDAPin);

/**
 * @brief   Retrieves flag status from the I2C_SR1 register
 * @param   pI2Cx	I2C Peripheral base address
 * @return  uint8_t	Flag set or reset
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flagName);

/**
 * Data send and receive
 */
/**
 * @brief	Send data
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @param	pTxBuffer	Pointer to the transmit buffer
 * @param	len			Size of the data we want to transmit
 * @param	slaveAddr	Address of the I2C Slave device
 * @param	Sr			Flag indicating whether the send data transaction will be followed by a repeated start
 * @return	void
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr);

/**
 * @brief	Receive data
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @param	pRxBuffer	Pointer to the transmit buffer
 * @param	len			Size of the data we want to transmit
 * @param	slaveAddr	Address of the I2C Slave device
 * @param	Sr			Flag indicating whether the receive data transaction will be followed by a repeated start
 * @return	void
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr);

/**
 * @brief	Interrupt driven send data
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @param	pTxBuffer	Pointer to the transmit buffer
 * @param	len			Size of the data we want to transmit
 * @param	slaveAddr	Address of the I2C Slave device
 * @param	Sr			Flag indicating whether the send data transaction will be followed by a repeated start
 * @return	uint8_t		I2C Application State
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr);

/**
 * @brief	Interrupt driven receive data
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @param	pRxBuffer	Pointer to the transmit buffer
 * @param	len			Size of the data we want to transmit
 * @param	slaveAddr	Address of the I2C Slave device
 * @param	Sr			Flag indicating whether the receive data transaction will be followed by a repeated start
 * @return	uint8_t		I2C Application State
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr);

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
 * @brief Handles interrupts generated on the I2C_EV line
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @return	void
 */
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle);

/**
 * @brief Handles interrupts generated on the I2C_ER line
 * @param	pI2CHandle 	I2C Peripheral Handle
 * @return	void
 */
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle);

/**
 * @brief	Generate the STOP condition
 * @param	pI2Cx		I2C peripheral base address
 * @return	void
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/**
 * @brief Close I2C peripheral data reception
 * @param 	pI2CHandle 	I2C Peripheral Handle
 * @return	void
 */
void I2C_CloseDataReception(I2C_Handle_t *pI2CHandle);

/**
 * @brief Close I2C peripheral data transmission
 * @param 	pI2CHandle 	I2C Peripheral Handle
 * @return	void
 */
void I2C_CloseDataTransmission(I2C_Handle_t *pI2CHandle);

/**
 * @brief	Send data in Slave mode
 * @param	pI2Cx	I2C peripheral base address
 * @param	data	Byte of data to send
 * @return	void
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);

/**
 * @brief	Receive data in Slave mode
 * @param	pI2Cx	I2C peripheral base address
 * @return	uint8_t	Byte of data receieved
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/**
 * Other Peripheral Control APIs
 */

/**
 * @brief 	Enable or disable the I2C peripheral
 * @note	You must configure and initialize the I2C peripheral before enabling it
 * @param	pI2Cx		I2C peripheral base address
 * @param	EnorDi		ENABLE or DISABLE macro
 * @return	void
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * @brief 	Enable or disable ACKing for the I2C peripheral
 * @param	pI2Cx		I2C peripheral base address
 * @param	EnorDi		ENABLE or DISABLE macro
 * @return	void
 */
void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * @brief Enable or disable interrupt callback events for the I2C peripheral
 * @param	pI2Cx		I2C peripheral base address
 * @param	EnorDi		ENABLE or DISABLE macro
 * @return	void
 */
void I2C_CallbackEventsControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

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

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
