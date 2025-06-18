#include "ds3231.h"
#include "i2c_driver.h"

/* DS3231 I2C slave address */
#define DS3231_SLAVE_ADDRESS    0x68

/* DS3231 Register Addresses */
#define DS3231_REG_SECONDS      0x00 /* Seconds Register. Bit 7: CH (Clock Halt) */
#define DS3231_REG_MINUTES      0x01 /* Minutes Register */
#define DS3231_REG_HOURS        0x02 /* Hours Register. Bit 6: 12/24 mode. Bit 5: AM/PM (if 12h mode) or 20h (if 24h mode) */
#define DS3231_REG_DAY          0x03 /* Day of the week Register (1-7) */
#define DS3231_REG_DATE         0x04 /* Date of the month Register (1-31) */
#define DS3231_REG_MONTH        0x05 /* Month Register. Bit 7: Century. (1-12) */
#define DS3231_REG_YEAR         0x06 /* Year Register (00-99) */


/**
 * @brief Convert BCD (Binary-Coded Decimal) to Decimal.
 * @param bcd_val BCD value to convert.
 * @return Decimal equivalent of the BCD value.
 * @example bcd_to_dec(0x23) returns 23.
 */
static uint8_t bcd_to_dec(uint8_t bcd_val) {
    return ((bcd_val >> 4) * 10) + (bcd_val & 0x0F);
}

/**
 * @brief Convert Decimal to BCD (Binary-Coded Decimal)
 * @param dec_val Decimal value to convert
 * @return BCD value
 */
static uint8_t dec_to_bcd(uint8_t dec_val) {
    return ((dec_val / 10) << 4) | (dec_val % 10);
}

/**
 * @brief Read a single byte from a specified register of the DS3231.
 * @param reg_addr The address of the DS3231 register to read from.
 * @return The byte value read from the register.
 */
static uint8_t DS3231_readRegister(uint8_t reg_addr) {
    uint8_t data;
    /* Generate START condition */
    I2C_Start();
    /* Send DS3231 slave address with WRITE bit */
    I2C_addressWrite(DS3231_SLAVE_ADDRESS);
    /* Send the register address to read from */
    I2C_writeByte(reg_addr);

    /* Generate REPEATED START condition */
    I2C_Start();
    /* Send DS3231 slave address with READ bit */
    I2C_addressRead(DS3231_SLAVE_ADDRESS);
    /* Read the byte from the register, send NACK for last byte */
    data = I2C_readByte(MULTI_BYTE_ACK_OFF);
    /* Generate STOP condition */
    I2C_Stop();

    return data;
}

/**
 * @brief Write a single byte to a specified register of the DS3231.
 * @param reg_addr The address of the DS3231 register to write to.
 * @param data The byte value to write to the register.
 */
static void DS3231_writeRegister(uint8_t reg_addr, uint8_t data) {
    I2C_Start();
    /* Send slave address with write bit */
    I2C_addressWrite(DS3231_SLAVE_ADDRESS);
    /* Send the register address to write to */
    I2C_writeByte(reg_addr);
    /* Write the data byte to the specified register */
    I2C_writeByte(data);
    /* Stop I2C communication */
    I2C_Stop();
}

/* Public Function Implementations */

/**
 * @brief Get the current seconds from the DS3231 RTC
 * @return Current seconds (0-59)
 */
uint8_t DS3231_getSeconds(void) {
    /* Read the seconds register from DS3231 */
    uint8_t bcd_seconds = DS3231_readRegister(DS3231_REG_SECONDS);
    /* The seconds register has a CH (Clock Halt) bit in bit 7 */
    return bcd_to_dec(bcd_seconds & 0x7F);
}

/**
 * @brief Get the current minutes from the DS3231 RTC
 * @return Current minutes (0-59)
 */
uint8_t DS3231_getMinutes(void) {
    /* Read the minutes register from DS3231 */
    uint8_t bcd_minutes = DS3231_readRegister(DS3231_REG_MINUTES);
    /* The minutes register does not have a CH bit, just convert BCD to decimal */
    return bcd_to_dec(bcd_minutes & 0x7F);
}

/**
 * @brief Get the current hours from the DS3231 RTC
 * @return Current hours (0-23 in 24-hour format)
 */
uint8_t DS3231_getHours(void) {
    uint8_t bcd_hours = DS3231_readRegister(DS3231_REG_HOURS);

    /* Bit 6 is 1: 12-hour format */
    if (bcd_hours & 0x40) { // 12-hour format indicator
        /* Mask 12/24h and AM/PM bits for basic BCD */
        return bcd_to_dec(bcd_hours & 0x1F);
    }
    /* Bit 6 is 0: 24-hour format */
    else {
        /* Mask upper two bits (unused in 24h mode, or bit 5 is 20h bit) */
        return bcd_to_dec(bcd_hours & 0x3F);
    }
}

/**
 * @brief Set the time on the DS3231 RTC
 * @param hh Hours (0-23)
 * @param mm Minutes (0-59)
 * @param ss Seconds (0-59)
 */
void DS3231_setTime(uint8_t hh, uint8_t mm, uint8_t ss) {
    I2C_Start();
    /* Set the DS3231 slave address and prepare to write */
    I2C_addressWrite(DS3231_SLAVE_ADDRESS);
    /* Start writing from seconds register */
    I2C_writeByte(DS3231_REG_SECONDS);

    /* Convert seconds, minutes, and hours to BCD format */
    I2C_writeByte(dec_to_bcd(ss));
    I2C_writeByte(dec_to_bcd(mm));

    /* 24 hour mode */
    I2C_writeByte(dec_to_bcd(hh) & 0x3F); /* Ensure bit 6 is 0 for 24hr mode */
    I2C_Stop();
}

/**
 * @brief Set the date on the DS3231 RTC
 * @param dow Day of the week (1-7, where 1 = Sunday)
 * @param date Day of the month (1-31)
 * @param month Month (1-12)
 * @param year Year (0-99, representing 2000-2099)
 */
void DS3231_setDate(uint8_t dow, uint8_t date, uint8_t month, uint8_t year) {
    I2C_Start();
    /* Set the DS3231 slave address and prepare to write */
    I2C_addressWrite(DS3231_SLAVE_ADDRESS);
    /* Start writing from day register */
    I2C_writeByte(DS3231_REG_DAY);

    /* Write DOW (DS3231 uses 1-7, ensure only lower 3 bits) */
    I2C_writeByte(dec_to_bcd(dow) & 0x07);
    /* Write date */
    I2C_writeByte(dec_to_bcd(date));
    /* Write month */
    I2C_writeByte(dec_to_bcd(month) & 0x1F);
    /* Write year */
    I2C_writeByte(dec_to_bcd(year));
    I2C_Stop();
}

/**
 * @brief Get the full time and date from the DS3231 RTC
 * @param time_struct Pointer to a ds3231_time_t structure to fill with current time and date
 * All values in the structure will be in decimal format.
 */
void DS3231_getFullTime(ds3231_time_t *time_struct) {
    uint8_t raw_seconds, raw_minutes, raw_hours, raw_day, raw_date, raw_month, raw_year;

    I2C_Start();
    /* Address DS3231 for write */
    I2C_addressWrite(DS3231_SLAVE_ADDRESS);
    /* Start reading from the seconds register (0x00) */
    I2C_writeByte(DS3231_REG_SECONDS);

    /* Repeated start for reading */
    I2C_Start();
    /* Address DS3231 for read */
    I2C_addressRead(DS3231_SLAVE_ADDRESS);

    /* Read all time/date registers in one go */
    raw_seconds = I2C_readByte(MULTI_BYTE_ACK_ON); /* Read seconds, ACK */
    raw_minutes = I2C_readByte(MULTI_BYTE_ACK_ON); /* Read minutes, ACK */
    raw_hours = I2C_readByte(MULTI_BYTE_ACK_ON);   /* Read hours, ACK */
    raw_day = I2C_readByte(MULTI_BYTE_ACK_ON);     /* Read day, ACK */
    raw_date = I2C_readByte(MULTI_BYTE_ACK_ON);    /* Read date, ACK */
    raw_month = I2C_readByte(MULTI_BYTE_ACK_ON);   /* Read month, ACK */
    raw_year = I2C_readByte(MULTI_BYTE_ACK_OFF);   /* Read year, NACK (last byte) */
    I2C_Stop();

    /* Convert BCD values to decimal and store in the structure */
    time_struct->seconds = bcd_to_dec(raw_seconds & 0x7F); /* Mask CH bit */
    time_struct->minutes = bcd_to_dec(raw_minutes & 0x7F); /* Mask unused bit 7 */

    /* Decode hours (handle 12/24 hour format) */
    if (raw_hours & 0x40) { /* Bit 6 is 1: 12-hour format */
        uint8_t am_pm = (raw_hours & 0x20) >> 5; /* Bit 5 is AM/PM: 0 for AM, 1 for PM */
        time_struct->hours = bcd_to_dec(raw_hours & 0x1F); /* Mask format and AM/PM bits */

        if (am_pm) { /* PM */
            if (time_struct->hours != 12) { /* 1 PM to 11 PM */
                time_struct->hours += 12;
            }
            /* else it's 12 PM (noon), which is 12 in 24h format */
        } else { /* AM */
            if (time_struct->hours == 12) { /* 12 AM (midnight) */
                time_struct->hours = 0;
            }
            /* else 1 AM to 11 AM are same in 24h format */
        }
    } else { /* Bit 6 is 0: 24-hour format */
        /* Mask potential 10hr/20hr bits if needed, generally fine for 0-23 */
        time_struct->hours = bcd_to_dec(raw_hours & 0x3F);
    }

    /* DOW is 1-7, mask other bits */
    time_struct->day = bcd_to_dec(raw_day & 0x07);
    /* Date is 1-31, mask other bits */
    time_struct->date = bcd_to_dec(raw_date & 0x3F);
    /* Mask out century bit (bit 7) */
    time_struct->month = bcd_to_dec(raw_month & 0x1F);
    /* Year is 00-99 */
    time_struct->year = bcd_to_dec(raw_year);
}
