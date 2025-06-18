#ifndef DS3231_H_
#define DS3231_H_

#include "stdint.h" /* For standard integer types like uint8_t */

/**
 * @brief Structure to hold time and date information from the DS3231 RTC.
 * All values are stored in decimal format.
 */
typedef struct {
    uint8_t seconds; /* Seconds: 0-59 */
    uint8_t minutes; /* Minutes: 0-59 */
    uint8_t hours;   /* Hours: 0-23 (24-hour format) */
    uint8_t day;     /* Day of the week: 1-7 (User can define mapping, e.g., Sunday=1 or as per DS3231 datasheet 1=Sunday) */
    uint8_t date;    /* Day of the month: 1-31 */
    uint8_t month;   /* Month: 1-12 */
    uint8_t year;    /* Year: 0-99 (representing 2000-2099, assuming century bit is handled or RTC is in 21st century) */
} ds3231_time_t;


/**
 * @brief Get the current seconds from the DS3231 RTC.
 * @return Current seconds (0-59) in decimal format.
 */
uint8_t DS3231_getSeconds(void);

/**
 * @brief Get the current minutes from the DS3231 RTC.
 * @return Current minutes (0-59) in decimal format.
 */
uint8_t DS3231_getMinutes(void);

/**
 * @brief Get the current hours from the DS3231 RTC.
 * @return Current hours (0-23 in 24-hour format) in decimal format.
 * This function attempts to convert 12-hour format (if set in RTC) to 24-hour format.
 */
uint8_t DS3231_getHours(void);

/**
 * @brief Set the time on the DS3231 RTC.
 * @param hh Hours (0-23, 24-hour format).
 * @param mm Minutes (0-59).
 * @param ss Seconds (0-59).
 * @note Time is set in 24-hour format on the DS3231.
 */
void DS3231_setTime(uint8_t hh, uint8_t mm, uint8_t ss);

/**
 * @brief Set the date on the DS3231 RTC.
 * @param dow Day of the week (1-7, e.g., 1 for Sunday).
 * @param date Day of the month (1-31).
 * @param month Month (1-12).
 * @param year Year (0-99, representing 2000-2099).
 * @note Century bit is not explicitly set; assumes 21st century.
 */
void DS3231_setDate(uint8_t dow, uint8_t date, uint8_t month, uint8_t year);

/**
 * @brief Get the full time and date from the DS3231 RTC.
 * @param time_struct Pointer to a ds3231_time_t structure to be filled with current time and date.
 * All values in the structure will be in decimal format.
 */
void DS3231_getFullTime(ds3231_time_t *time_struct);

#endif /* DS3231_H_ */
