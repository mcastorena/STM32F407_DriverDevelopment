/*
 * 011I2C_Master_Rx_Test.c
 *
 *  Created on: Jan 4, 2025
 *      Author: engineering
 */

#include "stm32f407xx_i2c_driver.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

#define I2C1_AF_MODE		4
#define MY_ADDR				0x61
#define SLAVE_ADDR			0x68
#define MAX_LEN				32

#define CMD_GET_LEN			0x51
#define CMD_RCV_DATA		0x52

uint8_t rxBuffer[MAX_LEN];

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioLed;

	// Set the base address to GPIO Port D
	gpioLed.pGPIOx = GPIOD;

	// Set pin configuration
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;		// The LED we are toggling is the green onboard LED connected to pin 12 on port D
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	// Set to push-pull output type
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioLed);

	GPIO_Handle_t gpioButton;

	// Set base address to GPIO port A
	gpioButton.pGPIOx = GPIOA;

	// Set pin configuration
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&gpioButton);
}

I2C_Handle_t I2C1Handle;
void I2C1_Init(void)
{
	/**
	 * PB6 --> I2C1_SCL
	 * PB7 --> I2C1_SDA
	 * ALT function mode: 4
	 */
	// Initialize the GPIO pins to be used for I2C1 peripheral
	I2C_GPIOInit(GPIOB, I2C1_AF_MODE, GPIO_PIN_NO_6, GPIO_PIN_NO_7);

	// Configure I2C Handle for I2C1
	memset(&I2C1Handle, 0, sizeof(I2C1Handle));

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;		// Doesn't really matter since we are in standard mode
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;		// 100kHz

	// Initialize SPI2
	I2C_Init(&I2C1Handle);
}

void waitForButtonInput()
{
	// Wait until the button is pressed
	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

	// Delay to avoid the button debouncing
	delay();

	// Toggle onboard LED for debugging
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

int main(void)
{
	// Initialize to use printf
	initialise_monitor_handles();
	printf("Application is running!\n");

	// Initialize the User button and onboard LED
	GPIO_ButtonInit();

	// Initialize the I2C1 peripheral
	I2C1_Init();

	// Enable the I2C1 Peripheral
	I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

	// Enable ACK if needed
	if(I2C1Handle.I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ACKControl(I2C1Handle.pI2Cx, I2C_ACK_ENABLE);
	}

	uint8_t cmdCode = 0;
	uint8_t rxLen = 0;

	while(1)
	{
		// Wait for button press
		waitForButtonInput();

		// Send the command to get the length of the data from the Arduino slave
		cmdCode = CMD_GET_LEN;
		I2C_MasterSendData(&I2C1Handle, &cmdCode, 1, SLAVE_ADDR);

		// Get the length from the slave
		I2C_MasterReceiveData(&I2C1Handle, &rxLen, 1, SLAVE_ADDR);
		printf("Length received from slave: %d\n", rxLen);

		// Send the command to receive the data from the slave
		cmdCode = CMD_RCV_DATA;
		I2C_MasterSendData(&I2C1Handle, &cmdCode, 1, SLAVE_ADDR);

		// Get the data from the slave
		I2C_MasterReceiveData(&I2C1Handle, rxBuffer, rxLen, SLAVE_ADDR);
		printf("Data received from slave: %s", rxBuffer);
	}

	return 0;
}
