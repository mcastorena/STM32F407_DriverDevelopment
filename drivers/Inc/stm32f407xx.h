/*
 * stm32f407xx.h
 *
 *  Created on: Dec 15, 2024
 *      Author: engineering
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

/**************************************************** Processor specific details **********************************************/
/**
 * ARM Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0			( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1			( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2			( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3			( (volatile uint32_t*)0xE000E10C )

/**
 * ARM Cortex M4 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0			( (volatile uint32_t*)0xE000E180 )
#define NVIC_ICER1			( (volatile uint32_t*)0xE000E184 )
#define NVIC_ICER2			( (volatile uint32_t*)0xE000E188 )
#define NVIC_ICER3			( (volatile uint32_t*)0xE000E18C )

/**
 * ARM Cortex M4 Processor NVIC IPRx register addresses
 */
#define NVIC_PR_BASEEADDR	( (volatile uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED 4

/**
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define ROM_BASEADDR		0x1FFF0000U		// System memory
#define	SRAM 				SRAM1_BASEADDR

/**
 * Base addresses of AHBx and APBx Peripherals
 */
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/**
 * Base addresses of peripherals hanging on the AHB1 Bus
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000)

#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/**
 * Base addresses of peripherals hanging on the APB1 Bus
 */
#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

/**
 * Base addresses of peripherals hanging on the APB2 Bus
 */
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)

/********************************** Peripheral Register definition structures **********************************/


/**
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	volatile uint32_t MODER;		// GPIO port mode register
	volatile uint32_t OTYPER;		// GPIO port output type register
	volatile uint32_t OSPEEDR;		// GPIO port output speed register
	volatile uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;			// GPIO port input data register
	volatile uint32_t ODR;			// GPIO port output data register
	volatile uint32_t BSRR;			// GPIO port bit set/reset register
	volatile uint32_t LCKR;			// GPIO port configuration lock register
	volatile uint32_t ARF[2];		// GPIO alternate function register Low:[0] and High:[1]
}GPIO_RegDef_t;

/**
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;			// SPI control register 1
	volatile uint32_t CR2;			// SPI control register 2
	volatile uint32_t SR;			// SPI status register
	volatile uint32_t DR;			// SPI data register
	volatile uint32_t CRCPR;		// SPI CRC polynomial register
	volatile uint32_t RXCRCR;		// SPI RX CRC register
	volatile uint32_t TXCRCR;		// SPI TX CRC register
	volatile uint32_t I2SCFGR;		// SPI_I2S configuration register
	volatile uint32_t I2SPR;		// SPI_I2S prescaler register
}SPI_RegDef_t;

/**
 * Peripheral register definition structure for I2C
 */
typedef struct
{
	volatile uint32_t CR1;			// I2C control register 1
	volatile uint32_t CR2;			// I2C control register 2
	volatile uint32_t OAR1;			// I2C Own address register 1
	volatile uint32_t OAR2;			// I2C Own address register 2
	volatile uint32_t DR;			// I2C Data register
	volatile uint32_t SR1;			// I2C Status register 1
	volatile uint32_t SR2;			// I2C Status register 2
	volatile uint32_t CCR;			// I2C Clock control register
	volatile uint32_t TRISE;		// I2C TRISE register
	volatile uint32_t FLTR;			// I2C FLTR register (only available on STM32F42xxx and STM32F43xxx)
}I2C_RegDef_t;

/**
 * Peripheral register definition structure for USART
 */
typedef struct
{
	volatile uint32_t SR;			// USART status register
	volatile uint32_t DR;			// USART data register
	volatile uint32_t BRR;			// USART baud rate register
	volatile uint32_t CR1;			// USART control register 1
	volatile uint32_t CR2;			// USART control register 2
	volatile uint32_t CR3;			// USART control register 3
	volatile uint32_t GTPR;			// USART guard time and prescaler register
}USART_RegDef_t;

/**
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	volatile uint32_t CR;			// RCC clock control register
	volatile uint32_t PLLCFGR;		// RCC PLL configuration register
	volatile uint32_t CFGR;			// RCC clock configuration register
	volatile uint32_t CIR;			// RCC clock interrupt register
	volatile uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register
	uint32_t RESERVED0;				// Reserved, 0x1C
	volatile uint32_t APB1RSTR;		// RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		// RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];			// Reserved, 0x28-0x2C
	volatile uint32_t AHB1ENR;		// RCC AHB1 peripheral clock register
	volatile uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;		// RCC AHB3 peripheral clock enable register
	uint32_t RESERVED2;				// Reserved, 0x3C
	volatile uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];			// Reserved, 0x48-0x4C
	volatile uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;	// RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;				// Reserved, 0x5C
	volatile uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	// RCC APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5[2];			// Reserved, 0x68-0x6C
	volatile uint32_t BDCR;			// RCC Backup domain control register
	volatile uint32_t CSR;			// RCC clock control & status register
	uint32_t RESERVED6[2];			// Reserved, 0x78-0x7C
	volatile uint32_t SSCGR;		// RCC spread spectrum clock generation register
	volatile uint32_t PLLI2CCFGR;	// RCC PLLI2S configuration register
	volatile uint32_t PLLSAICFGR;	// RCC PLL configuration register
	volatile uint32_t DCKCRFGR;		// RCC Dedicated Clock Configuration Register
}RCC_RegDef_t;

/**
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t IMR;			// Interrupt mask register
	volatile uint32_t EMT;			// Event mask register
	volatile uint32_t RTSR;			// Rising trigger selection register
	volatile uint32_t FTSR;			// Falling trigger selection register
	volatile uint32_t SWIER;		// Software interrupt event register
	volatile uint32_t PR;			// Pending register
}EXTI_RegDef_t;

/**
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRMP;		// Memory remap register
	volatile uint32_t PMC;			// Peripheral mode configuration register
	volatile uint32_t EXTICR[4];	// External interrupt configuration register 1 thru 4
	uint32_t RESERVED1[2];			// Reserved, 0x18-0x1C
	volatile uint32_t CMPCR;		// Compensation cell control register
	uint32_t RESERVED2[2];			// Reserved, 0x24-0x28
	volatile uint32_t CFGR;			//
}SYSCFG_RegDef_t;

/**
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/**
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 8 ) )

/**
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )

/**
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )

/**
 * Clock Enable Macros for UARTx/USARTx peripherals
 */
#define USART1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 5 ) )

/**
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 14 ) )

/**
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 8 ) )

/**
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )

/**
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )

/**
 * Clock Disable Macros for UARTx/USARTx peripherals
 */
#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 18 ) )
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 19 ) )
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 20 ) )
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 5 ) )


/**
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 14 ) )

/**
 * Macros to reset GPIOx peripherals
 * Sets then clears the reset register bit field for the GPIO port
 */
#define GPIOA_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 0) );	( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 1) );	( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 2) );	( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 3) );	( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 4) );	( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOF_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 5) );	( RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)
#define GPIOG_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 6) );	( RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)
#define GPIOH_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 7) );	( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)
#define GPIOI_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 8) );	( RCC->AHB1RSTR &= ~(1 << 8) ); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0:\
									(x == GPIOB) ? 1:\
									(x == GPIOC) ? 2:\
									(x == GPIOD) ? 3:\
									(x == GPIOE) ? 4:\
									(x == GPIOF) ? 5:\
									(x == GPIOG) ? 6:\
									(x == GPIOH) ? 7:\
									(x == GPIOI) ? 8:0 )
/**
 * Macros to define the IRQ numbers for EXTI lines
 */
#define IRQ_NO_EXT0				6
#define IRQ_NO_EXT1				7
#define IRQ_NO_EXT2				8
#define IRQ_NO_EXT3				9
#define IRQ_NO_EXT4				10
#define IRQ_NO_EXT9_5			23
#define IRQ_NO_EXT15_10			40

/**
 * Macros to define the IRQ numbers for SPI interrupt lines
 */
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

/**
 * Macros to define the IRQ numbers for I2C interrupt lines
 */
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32

#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34

#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

/**
 * Macros to define IRQ priority levels
 */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15

/**
 * Some generic macros
 */
#define ENABLE			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET

/********************************** Bit position macro definitions for SPI peripheral ********************/

/**
 * SPI_CR1 register bit position macros
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSF_FIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RX_ONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDI_OE			14
#define SPI_CR1_BIDI_MODE		15

/**
 * SPI_CR2 register bit position macros
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/**
 * SPI_SR register bit position macros
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/********************************** Bit position macro definitions for I2C peripheral ********************/

/**
 * I2C_CR1 register bit position macros
 */
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRECH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

/**
 * I2C_CR2 register bit position macros
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

/**
 * I2C_OAR1 register bit position macros
 */
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD71			1
#define I2C_OAR1_ADD98			8
#define I2C_OAR1_ADDMODE		15

/**
 * I2C_SR1 register bit position macros
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

/**
 * I2C_SR2 register bit position macros
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

/**
 * I2C_CCR register bit position macros
 */
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15

/********************************** Bit position macro definitions for USART peripheral ********************/

/**
 * USART_CR1 register bit position macros
 */
#define USART_CR1_SBK			0
#define USART_CR1_RWU 			1
#define USART_CR1_RE  			2
#define USART_CR1_TE 			3
#define USART_CR1_IDLEIE 		4
#define USART_CR1_RXNEIE  		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE 			8
#define USART_CR1_PS 			9
#define USART_CR1_PCE 			10
#define USART_CR1_WAKE  		11
#define USART_CR1_M 			12
#define USART_CR1_UE 			13
#define USART_CR1_OVER8  		15

/*
 * USART_CR2 register bit position macros
 */
#define USART_CR2_ADD   		0
#define USART_CR2_LBDL   		5
#define USART_CR2_LBDIE  		6
#define USART_CR2_LBCL   		8
#define USART_CR2_CPHA   		9
#define USART_CR2_CPOL   		10
#define USART_CR2_STOP   		12
#define USART_CR2_LINEN   		14


/*
 * USART_CR3 register bit position macros
 */
#define USART_CR3_EIE   		0
#define USART_CR3_IREN   		1
#define USART_CR3_IRLP  		2
#define USART_CR3_HDSEL   		3
#define USART_CR3_NACK   		4
#define USART_CR3_SCEN   		5
#define USART_CR3_DMAR  		6
#define USART_CR3_DMAT   		7
#define USART_CR3_RTSE   		8
#define USART_CR3_CTSE   		9
#define USART_CR3_CTSIE   		10
#define USART_CR3_ONEBIT   		11

/**
 * USART_SR register bit position macros
 */
#define USART_SR_PE        		0
#define USART_SR_FE        		1
#define USART_SR_NE        		2
#define USART_SR_ORE       		3
#define USART_SR_IDLE       	4
#define USART_SR_RXNE        	5
#define USART_SR_TC        		6
#define USART_SR_TXE        	7
#define USART_SR_LBD        	8
#define USART_SR_CTS        	9


#endif /* INC_STM32F407XX_H_ */
