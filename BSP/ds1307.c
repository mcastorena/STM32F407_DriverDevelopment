/*
 * ds1307.c
 *
 *  Created on: Jan 25, 2025
 *      Author: engineering
 */
#include "ds1307.h"

I2C_Handle_t g_DS1307_I2CHandle;

/**
 * Private function prototypes
 */
static void DS1307_I2C_PinConfiguration(void);
static void DS1307_I2C_Configuration(void);
static void DS1307_Write(uint8_t value, uint8_t regAddress);
static uint8_t DS1307_Read(uint8_t regAddress);
static uint8_t binaryToBCD(uint8_t value);
static uint8_t BCDToBinary(uint8_t value);

uint8_t DS1307_Init(void)
{
	/**
	 * Init DS1307 I2C peripheral
	 */
	// Initialize the I2C Pins
	DS1307_I2C_PinConfiguration();

	// Initialize the I2C peripherals
	DS1307_I2C_Configuration();

	// Enable the I2C Peripheral
	I2C_PeripheralControl(g_DS1307_I2CHandle.pI2Cx, ENABLE);

	// Write 0 to the Clock Halt bit field to enable the oscillator
	DS1307_Write(0x00, DS1307_ADDR_SECS);

	// Read back Clock Halt bit
	uint8_t clockState = DS1307_Read(DS1307_ADDR_SECS);

	// Return clock state to the application to determine if initialization was successful
	return (1 & (clockState >> 7));
} // DS1307_Init

void DS1307_SetCurrentTime(RTC_Time_t *rtcTime)
{
	// Seconds
	uint8_t seconds = 0;
	seconds = binaryToBCD(rtcTime->seconds);
	seconds &= ~( 1 << 7 );		// Clear the 7th bit to ensure the Clock Halt bit does not stop the oscillator
	DS1307_Write(seconds, DS1307_ADDR_SECS);

	// Minutes
	uint8_t minutes = 0;
	minutes = binaryToBCD(rtcTime->minutes);
	DS1307_Write(minutes, DS1307_ADDR_MIN);

	// Hours
	uint8_t hours = 0;
	hours = binaryToBCD(rtcTime->hours);
	if(rtcTime->time_format == TIME_FORMAT_24HRS)
	{
		// Clear the 6th bit for 24hr time format
		hours &= ~( 1 << 6 );
	}
	else
	{
		// Set the 6th bit for 12hr time format
		hours |= ( 1 << 6 );

		// Set or clear the 6th bit for AM/PM
		hours = (rtcTime->time_format == TIME_FORMAT_12HRS_PM) ? ( hours | ( 1 << 5 ) ) : ( hours & ~( 1 << 5 ));
	}
	DS1307_Write(hours, DS1307_ADDR_HR);
} // DS1307_SetCurrentTime

void DS1307_GetCurrentTime(RTC_Time_t *rtcTime)
{
	// Seconds
	uint8_t seconds = DS1307_Read(DS1307_ADDR_SECS);
	seconds &= ~( 1 << 7 );		// Clear the 7th bit to ensure the Clock Halt bit does affect the data read
	rtcTime->seconds = BCDToBinary(seconds);

	// Minutes
	uint8_t minutes = DS1307_Read(DS1307_ADDR_MIN);
	rtcTime->minutes = BCDToBinary(minutes);

	// Hours
	uint8_t hours = DS1307_Read(DS1307_ADDR_HR);
	if(hours & ( 1 << 6 ))	// Get the value of the 6th bit to determine if we are in 24hr or 12hr time format
	{
		//12hr format
		rtcTime->time_format =  (hours & ( 1 << 5 )) ? TIME_FORMAT_12HRS_PM : TIME_FORMAT_12HRS_AM; // Check the AM/PM bitfield
		hours &= ~(0x3 << 5);//Clear 6 and 5
	}
	else
	{
		//24hr format
		rtcTime->time_format = TIME_FORMAT_24HRS;
	}
	rtcTime->hours = BCDToBinary(hours);
} // DS1307_SetCurrentTime

void DS1307_SetCurrentDate(RTC_Date_t *rtcDate)
{
	// Date
	DS1307_Write(binaryToBCD(rtcDate->date), DS1307_ADDR_DATE);

	// Month
	DS1307_Write(binaryToBCD(rtcDate->month), DS1307_ADDR_MONTH);

	// Year
	DS1307_Write(binaryToBCD(rtcDate->year), DS1307_ADDR_YEAR);

	// Day
	DS1307_Write(binaryToBCD(rtcDate->day), DS1307_ADDR_DAY);

} // DS1307_SetCurrentDate

void DS1307_GetCurrentDate(RTC_Date_t *rtcDate)
{
	// Date
	rtcDate->date = BCDToBinary(DS1307_Read(DS1307_ADDR_DATE));

	// Month
	rtcDate->month = BCDToBinary(DS1307_Read(DS1307_ADDR_MONTH));

	// Year
	rtcDate->year = BCDToBinary(DS1307_Read(DS1307_ADDR_YEAR));

	// Day
	rtcDate->day = BCDToBinary(DS1307_Read(DS1307_ADDR_DAY));
} // DS1307_GetCurrentDate

/**
 * Configure the GPIO pins for I2C usage by DS1307
 */
static void DS1307_I2C_PinConfiguration(void)
{
	GPIO_Handle_t I2C_SDA, I2C_SCL;
	memset(&I2C_SDA, 0, sizeof(I2C_SCL));
	memset(&I2C_SCL, 0, sizeof(I2C_SDA));

	// SDA Init
	I2C_SDA.pGPIOx = DS1307_I2C_GPIO_PORT;
	I2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode = DS1307_I2C_GPIO_AF_MODE;
	I2C_SDA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C_SDA.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	I2C_SDA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C_SDA.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	I2C_SDA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&I2C_SDA);

	// SCL Init
	I2C_SCL.pGPIOx = DS1307_I2C_GPIO_PORT;
	I2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode = DS1307_I2C_GPIO_AF_MODE;
	I2C_SCL.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C_SCL.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	I2C_SCL.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C_SCL.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	I2C_SCL.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&I2C_SCL);
} // DS1307_I2C_PinConfiguration

/**
 * Initializes I2C Peripheral for usage by DS1307
 */
static void DS1307_I2C_Configuration(void)
{
	memset(&g_DS1307_I2CHandle, 0, sizeof(g_DS1307_I2CHandle));

	g_DS1307_I2CHandle.pI2Cx = DS1307_I2C;
	g_DS1307_I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_DS1307_I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	g_DS1307_I2CHandle.I2C_Config.I2C_DeviceAddress = DS1307_I2C_ADDR;
	I2C_Init(&g_DS1307_I2CHandle);
} // DS1307_I2C_Configuration

static void DS1307_Write(uint8_t value, uint8_t regAddress)
{
	uint8_t tx[2];
	tx[0] = regAddress;
	tx[1] = value;
	I2C_MasterSendData(&g_DS1307_I2CHandle, tx, 2, g_DS1307_I2CHandle.I2C_Config.I2C_DeviceAddress, 0);
} // DS1307_Write

static uint8_t DS1307_Read(uint8_t regAddress)
{
	// Perform a write to set the address pointer in the DS1307 chip
	I2C_MasterSendData(&g_DS1307_I2CHandle, &regAddress, 1, g_DS1307_I2CHandle.I2C_Config.I2C_DeviceAddress, 0);

	// Read from the register address in the DS1307
	uint8_t data = 0;
	I2C_MasterReceiveData(&g_DS1307_I2CHandle, &data, 1, g_DS1307_I2CHandle.I2C_Config.I2C_DeviceAddress, 0);

	return data;
} // DS1307_Read

/**
 * Convert binary to BCD for values less than 100
 */
static uint8_t binaryToBCD(uint8_t value)
{
	uint8_t m,n;
	uint8_t bcd = value;
	if(value >= 10)
	{
		m = value/10;
		n = value%10;
		bcd = (uint8_t)( ( m << 4 ) | n );
	}
	return bcd;
} // binaryToBCD

/**
 * Convert BCD to binary for values less than 100
 */
static uint8_t BCDToBinary(uint8_t value)
{
	uint8_t m,n;
	uint8_t binary = 0;

	m = ( (uint8_t)( value >> 4 ) * 10 );
	n = ( value & (uint8_t)0x0F );
	binary = (m+n);

	return binary;
} // BCDToBinary
