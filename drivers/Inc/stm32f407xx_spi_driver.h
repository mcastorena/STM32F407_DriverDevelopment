/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Dec 23, 2024
 *      Author: engineering
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/**
 * Configuration structure for a SPI peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/**
 * Handle structure for a SPI peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;			// Holds the base address of the SPIx(x:0,1,2) peripheral
	SPI_Config_t 	SPIConfig;		// Holds SPIx peripheral configuration settings
	uint8_t			*pTxBuffer;		// Stores the application TX Buffer address
	uint8_t			*pRxBuffer;		// Stores the application RX Buffer address
	uint32_t		txLen;			// Length of TX Buffer
	uint32_t		rxLen;			// Length of RX Buffer
	uint8_t			txState;
	uint8_t			rxState;
}SPI_Handle_t;

/**
 * SPI Application States
 */
#define SPI_READY				0
#define SPI_BUSY_IN_TX			1
#define SPI_BUSY_IN_RX			2

/**
 * SPI Application Events
 */
#define SPI_EVENT_TX_COMPLETE	1
#define SPI_EVENT_RX_COMPLETE	2
#define SPI_EVENT_OVR_COMPLETE	3

/**
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/**
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1		// Full duplex
#define SPI_BUS_CONFIG_HD				2		// Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3		// Simplex, rx only

/**
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/**
 * @SPI_DFF
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/**
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

/**
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

/**
 * @SPI_SSM
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0

/**
 * SPI Status Flag definitions
 */
#define SPI_TXE_FLAG					( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG					( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG					( 1 << SPI_SR_BSY )
#define SPI_OVR_FLAG					( 1 << SPI_SR_OVR )

#define SPI_TXEIE_FLAG					( 1 << SPI_CR2_TXEIE )
#define SPI_RXNEIE_FLAG					( 1 << SPI_CR2_RXNEIE )
#define SPI_ERRIE_FLAG					( 1 << SPI_CR2_ERRIE )

/*********************** APIs supported by this driver *********************************/

/**
 * Peripheral Clock setup
 */

/**
 * @brief   Enable/Disable peripheral clock for the given SPI peripheral
 * @param   pGPIOx    SPI Peripheral base address
 * @param   EnorDi    ENABLE or DISABLE macro
 * @return  void
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * Init and De-init
 */
/**
 * @brief   Initialize the given SPI peripheral
 * @param   pSPIHandle    SPI Handle
 * @return  void
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @brief   De-initialize the given SPI peripheral
 * @param   pSPIx    SPI Peripheral base address
 * @return  void
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief   Initializes the GPIO pins for SPI usage
 * @param   pGPIOx		GPIO Peripheral base address
 * @param   AFMode		Alternate Function mode
 * @param	MOSIPin		Pin number for SPI MOSI
 * @param	MISOPin		Pin number for SPI MISO
 * @param	SCLKPin		Pin number for SPI SCLK
 * @param	NSSPin		Pin number for SPI NSS
 * @return  void
 */
void SPI_GPIOInit(GPIO_RegDef_t *pGPIOx, uint8_t AFMode, uint8_t MOSIPin, uint8_t MISOPin, uint8_t SCLKPin, uint8_t NSSPin);

/**
 * @brief   Retrieves flag status from the SPI_SR register
 * @param   pSPIx	SPI Peripheral base address
 * @return  uint8_t	Flag set or reset
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName);

/**
 * Data send and receive
 */
/**
 * @brief	Send data
 * @note	This is a blocking call
 * @param   pSPIx    	SPI Peripheral base address
 * @param	pTxBuffer	Pointer to the transmit buffer
 * @param	len			Size of the data we want to transmit
 * @return  void
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief	Read data
 * @param   pSPIx    	SPI Peripheral base address
 * @param	pRxBuffer	Pointer to the receive buffer
 * @param	len			Size of the data we want to receive
 * @return  void
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief	Interrupt-based send data
 * @note	This is a blocking call
 * @param   pSPIHandle 	SPI Peripheral Handle
 * @param	pTxBuffer	Pointer to the transmit buffer
 * @param	len			Size of the data we want to transmit
 * @return  uint8_t		SPI Peripheral TX state
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief	Interrupt-based read data
 * @param   pSPIHandle 	SPI Peripheral Handle
 * @param	pRxBuffer	Pointer to the receive buffer
 * @param	len			Size of the data we want to receive
 * @return  uint8_t		SPI Peripheral RX state
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/**
 * IRQ configuration and ISR handling
 */
/**
 * @brief	Enable or disable the given IRQ number
 * @param   IRQNumber	IRQ number
 * @param	EnorDi		GPIO pin number
 * @return	void
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief	Set the IRQ priority level for the given IRQ number
 * @param   IRQNumber		IRQ number
 * @param	IRQPriority		Priority level from 0 to 15
 * @return	void
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief 	Handle an interrupt for the SPI peripheral
 * @param	pSPIHandle 	SPI Peripheral Handle
 * @return	void
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/**
 * Other Peripheral Control APIs
 */

/**
 * @brief 	Enable or disable the SPI peripheral
 * @note	You must configure and initialize the SPI peripheral before enabling it
 * @param	pSPIx		SPI peripheral base address
 * @param	EnorDi		ENABLE or DISABLE macro
 * @return	void
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief 	Enable or disable the SPI peripheral internal slave select (SSI bit in CR1)
 * @note	This must be set HIGH when SSM is HIGH
 * @param	pSPIx		SPI peripheral base address
 * @param	EnorDi		ENABLE or DISABLE macro
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief 	Enable or disable the SPI peripheral slave select output enable (SSOE bit in CR2)
 * @note	This must be set HIGH when SSM is LOW in order to select the slave
 * @param	pSPIx		SPI peripheral base address
 * @param	EnorDi		ENABLE or DISABLE macro
 * @return	void
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief	Clear the OVR flag by reading from the DR and SR
 * @param	pSPIx		SPI peripheral base address
 * @return	void
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

/**
 * @brief	Close SPI peripheral transmission
 * @param	pSPIHandle 	SPI Peripheral Handle
 * @return	void
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

/**
 * @brief	Close SPI peripheral reception
 * @param	pSPIHandle 	SPI Peripheral Handle
 * @return	void
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/**
 * Application call-back
 */
/**
 * @brief	Application callback function
 * @note	This is a weak implementation, it must be overridden by
 * 			the application to suit its requirements
 * @param	pSPIHandle 	SPI Peripheral Handle
 * @param	appEv		Application event macro
 * @return	void
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
