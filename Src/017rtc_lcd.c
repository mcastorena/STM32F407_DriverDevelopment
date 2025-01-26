/*
 * 017rtc_lcd.c
 *
 *  Created on: Jan 25, 2025
 *      Author: engineering
 */
#include <stdio.h>
#include "ds1307.h"

/**
 * Converts integers less than 100 to their ASCII value
 */
void intToStr(uint8_t num, char* buf)
{
	if(num < 10)
	{
		buf[0] = '0';
		buf[1] = num + 48;
	}
	else
	{
		buf[0] = ( (num / 10) + 48 );
		buf[1] = ( (num % 10) + 48 );
	}
}

/**
 * Returns string of the day of the week
 */
char* getDayOfWeek(uint8_t day)
{
	char* daysOfWeek[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

	return daysOfWeek[day-1];
} // getDayOfWeek

/**
 * Returns time in format HH:MM:SS
 */
char* timeToString(RTC_Time_t *rtcTime)
{
	static char buf[9];

	// Insert colon characters
	buf[2] = ':';
	buf[5] = ':';

	// Convert hours, minutes, seconds
	intToStr(rtcTime->hours, buf);
	intToStr(rtcTime->minutes, &buf[3]);
	intToStr(rtcTime->seconds, &buf[6]);

	// Insert termination character
	buf[8] = '\0';

	return buf;
}

/**
 * Returns date in format DD/MM/YY
 */
char* dateToString(RTC_Date_t *rtcDate)
{
	static char buf[9];

	// Insert / characters
	buf[2] = '/';
	buf[5] = '/';

	// Convert hours, minutes, seconds
	intToStr(rtcDate->date, buf);
	intToStr(rtcDate->month, &buf[3]);
	intToStr(rtcDate->year, &buf[6]);

	// Insert termination character
	buf[8] = '\0';

	return buf;
} // dateToString

extern void initialise_monitor_handles();

int main(void)
{
	// Initialize to use printf
	initialise_monitor_handles();
	printf("RTC Test\n");

	if(DS1307_Init())
	{
		printf("RTC initialization failed!\n");
		while(1);
	}

	/**
	 * Set the current date and time
	 */
	RTC_Date_t currentDate;
	currentDate.date = 25;
	currentDate.day = SATURDAY;
	currentDate.month = 1;
	currentDate.year = 25;
	DS1307_SetCurrentDate(&currentDate);

	RTC_Time_t currentTime;
	currentTime.hours = 6;
	currentTime.minutes = 46;
	currentTime.seconds = 22;
	currentTime.time_format = TIME_FORMAT_12HRS_PM;
	DS1307_SetCurrentTime(&currentTime);

	/**
	 * Verify by reading the values
	 */
	DS1307_GetCurrentDate(&currentDate);
	DS1307_GetCurrentTime(&currentTime);

	if(currentTime.time_format != TIME_FORMAT_24HRS)
	{
		char *amPM = (currentTime.time_format) ? "PM" : "AM";
		printf("Current time: %s %s\n", timeToString(&currentTime), amPM);
	}
	else
	{
		printf("Current time: %s\n", timeToString(&currentTime));
	}

	printf("Current date: %s %s\n", dateToString(&currentDate), getDayOfWeek(currentDate.day));

	while(1);

	return 0;
}
