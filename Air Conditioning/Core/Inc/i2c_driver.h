#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "stm32f4xx.h"
#include "stdint.h"

/**
 * @brief Enumeration to specify whether to send an ACK or NACK after a byte reception.
 * This is typically used when reading multiple bytes from a slave device.
 */
typedef enum {
    /* Send NACK (No Acknowledge) - typically for the last byte of a reception. */
    MULTI_BYTE_ACK_OFF = 0,
    /* Send ACK (Acknowledge) - typically when more bytes are expected to be received. */
    MULTI_BYTE_ACK_ON
} ack_status_t;

/**
 * @brief Initialize I2C1 peripheral.
 * @note This function configures GPIO pins PB6 (SCL) and PB7 (SDA) for I2C1.
 * It sets up I2C timing for standard mode (100kHz) assuming PCLK1 is 42MHz.
 * It also includes a routine to attempt recovery from a stuck I2C bus.
 */
void I2C_Init(void);

/**
 * @brief Generate an I2C START condition on the bus.
 * @note This function also enables ACKing from the master side.
 * It waits until the START condition is successfully generated (SB flag is set).
 */
void I2C_Start(void);

/**
 * @brief Generate an I2C STOP condition on the bus.
 * @note It waits until the STOP condition is generated and the bus is no longer busy.
 */
void I2C_Stop(void);

/**
 * @brief Send a slave address with the WRITE bit (0).
 * @param slave_address The 7-bit slave address.
 * @note This function waits until the address is sent and acknowledged by the slave (ADDR flag is set).
 * It then clears the ADDR flag.
 */
void I2C_addressWrite(uint8_t slave_address);

/**
 * @brief Send a slave address with the READ bit (1).
 * @param slave_address The 7-bit slave address.
 * @note This function waits until the address is sent and acknowledged by the slave (ADDR flag is set).
 * It then clears the ADDR flag.
 */
void I2C_addressRead(uint8_t slave_address);

/**
 * @brief Write a single byte of data to the I2C bus.
 * @param byte The byte of data to send.
 * @note This function waits for the Transmit data register Empty (TXE) flag before writing,
 * and then waits for the Byte Transfer Finished (BTF) flag to ensure transmission completion.
 */
void I2C_writeByte(uint8_t byte);

/**
 * @brief Read a single byte of data from the I2C bus.
 * @param ack Specifies whether to send an ACK or NACK after receiving the byte.
 * Use MULTI_BYTE_ACK_ON if more bytes are to be read.
 * Use MULTI_BYTE_ACK_OFF if this is the last byte to be read.
 * @return The received byte.
 * @note This function waits for the Receive data register Not Empty (RXNE) flag before reading.
 */
uint8_t I2C_readByte(ack_status_t ack);

#endif /* I2C_DRIVER_H_ */
