/*
 * ds1307.h
 *
 *  Created on: Jan 25, 2025
 *      Author: engineering
 */

#ifndef DS1307_H_
#define DS1307_H_

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

/**
 * Application configurable items
 */
#define DS1307_I2C					I2C1
#define DS1307_I2C_GPIO_PORT		GPIOB
#define DS1307_I2C_GPIO_AF_MODE		4					// For PB6 and 7
#define DS1307_I2C_SDA_PIN			GPIO_PIN_NO_7		// PB7
#define DS1307_I2C_SCL_PIN			GPIO_PIN_NO_6		// PB6
#define DS1307_I2C_SPEED			I2C_SCL_SPEED_SM	// Standard mode, does not support fast mode
#define DS1307_I2C_PUPD				GPIO_PIN_PU			// We can also use external 3.3kOhm pull-up resistors

/**
 * Register addresses
 */
#define DS1307_ADDR_SECS			0x00
#define DS1307_ADDR_MIN				0x01
#define DS1307_ADDR_HR				0x02
#define DS1307_ADDR_DAY				0x03
#define DS1307_ADDR_DATE			0x04
#define DS1307_ADDR_MONTH			0x05
#define DS1307_ADDR_YEAR			0x06

#define DS1307_I2C_ADDR				0x68

/**
 * Time formats
 */
#define TIME_FORMAT_12HRS_AM		0
#define TIME_FORMAT_12HRS_PM		1
#define TIME_FORMAT_24HRS			2

#define SUNDAY						1;
#define MONDAY						2;
#define TUESDAY						3;
#define WEDNESDAY					4;
#define THURSDAY					5;
#define FRIDAY						6;
#define SATURDAY					7;

typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_Date_t;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_Time_t;

/**
 * Function prototypes
 */

uint8_t DS1307_Init(void);

void DS1307_SetCurrentTime(RTC_Time_t *rtcTime);
void DS1307_GetCurrentTime(RTC_Time_t *rtcTime);

void DS1307_SetCurrentDate(RTC_Date_t *rtcDate);
void DS1307_GetCurrentDate(RTC_Date_t *rtcDate);

#endif /* DS1307_H_ */
