/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Dec 23, 2024
 *      Author: engineering
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

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
	SPI_RegDef_t *pSPIx;		// Holds the base address of the SPIx(x:0,1,2) peripheral
	SPI_Config_t SPIConfig;		// Holds SPIx peripheral configuration settings
}SPI_Handle_t;

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
 * @brief
 * @param   pSPIx    	SPI Peripheral base address
 * @param	pRxBuffer	Pointer to the receieve buffer
 * @param	len			Size of the data we want to receive
 * @return  void
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/**
 * IRQ configuration and ISR handling
 */
/**
 * @brief	Enable or disable the given IRQ number
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief	Set the IRQ priority level for the given IRQ number
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief 	Handle an interrupt for the SPI peripheral
 * @param	pSPIx		SPI peripheral base address
 */
void SPI_IRQHandling(SPI_RegDef_t *pSPIx);

/**
 * Other Peripheral Control APIs
 */


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
