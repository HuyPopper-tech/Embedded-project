#include "i2c_driver.h"

#define PCLK1_FREQ_MHZ         42                   /* PCLK1 frequency in MHz, adjust as needed */
#define I2C_STANDARD_MODE_CCR  (PCLK1_FREQ_MHZ * 5) /* Standard mode CCR value for 100kHz */
#define I2C_TRISE_VALUE        (PCLK1_FREQ_MHZ + 1) /* TRISE value for 100kHz */

#define I2C_GPIO_RCC_ENR RCC_AHB1ENR_GPIOBEN  /* GPIOB clock enable bit */
#define I2C_RCC_ENR      RCC_APB1ENR_I2C1EN   /* I2C1 clock enable bit */
#define I2C_PORT         GPIOB                /* I2C GPIO port */
#define I2C_SCL_PIN      6                    /* SCL pin number */
#define I2C_SDA_PIN      7                    /* SDA pin number */

/**
 * @brief Initialize I2C1 peripheral for communication
 * This function configures the GPIO pins for I2C,
 * enables the I2C1 peripheral,
 * and sets up the I2C timing for standard mode (100kHz).
 * It also handles the BUSY flag to ensure
 * the I2C bus is not stuck before initialization.
 * * @note This function assumes the system clock is configured
 */
void I2C_Init(void) {
    /* Enable clocks for GPIO and I2C peripherals */
    RCC->AHB1ENR |= I2C_GPIO_RCC_ENR;
    RCC->APB1ENR |= I2C_RCC_ENR;

    /* 2. Configure PB6 (SCL) and PB7 (SDA) for I2C1 alternate function */
    /* These pins need to be set to: Alternate function, Open-drain, Pull-up, High speed. */

    /* Clear mode bits for PB6 and PB7 */
    I2C_PORT->MODER &= ~((3U << (I2C_SCL_PIN * 2)) | (3U << (I2C_SDA_PIN * 2)));
    /* Set PB6 and PB7 to Alternate function mode (10) */
    I2C_PORT->MODER |= (2U << (I2C_SCL_PIN * 2)) | (2U << (I2C_SDA_PIN * 2));

    /* Set PB6 and PB7 to Output open-drain (1) */
    I2C_PORT->OTYPER |= (1U << I2C_SCL_PIN) | (1U << I2C_SDA_PIN);

    /* Set PB6 and PB7 to High speed (11) */
    I2C_PORT->OSPEEDR |= (3U << (I2C_SCL_PIN * 2)) | (3U << (I2C_SDA_PIN * 2));

    /* Clear pull-up/pull-down bits for PB6 and PB7 */
    I2C_PORT->PUPDR &= ~((3U << (I2C_SCL_PIN * 2)) | (3U << (I2C_SDA_PIN * 2)));
    /* Enable Pull-up resistors for PB6 and PB7 (01) */
    I2C_PORT->PUPDR |= (1U << (I2C_SCL_PIN * 2)) | (1U << (I2C_SDA_PIN * 2));

    /* Configure Alternate Function for PB6 and PB7 to I2C1 (AF4) */
    /* AFR[0] is for pins 0-7. PB6 is pin 6, PB7 is pin 7. */
    /* Clear AF selection bits for PB6 and PB7 */
    I2C_PORT->AFR[0] &= ~((0xFU << (I2C_SCL_PIN * 4))
            | (0xFU << (I2C_SDA_PIN * 4)));
    /* Set AF4 (I2C1) for PB6 and PB7 */
    I2C_PORT->AFR[0] |= (4U << (I2C_SCL_PIN * 4)) | (4U << (I2C_SDA_PIN * 4));

    /* 3. Check and handle I2C BUSY flag (optional, but good practice for robustness) */
    /* If the BUSY flag is set, it might indicate a previously stuck communication. */
    /* This attempts to free the bus by manually generating clock pulses and a STOP condition. */
    if (I2C1->SR2 & I2C_SR2_BUSY) {
        /* Temporarily configure I2C pins (PB6 SCL, PB7 SDA) as GPIO outputs */
        I2C_PORT->MODER &= ~((3U << (I2C_SCL_PIN * 2))
                | (3U << (I2C_SDA_PIN * 2)));
        I2C_PORT->MODER |= (1U << (I2C_SCL_PIN * 2))
                | (1U << (I2C_SDA_PIN * 2));  // Output mode
        I2C_PORT->ODR |= (1U << I2C_SCL_PIN) | (1U << I2C_SDA_PIN); // Both SDA and SCL high

        /* Short delay for lines to stabilize */
        for (volatile int i = 0; i < 1000; i++)
            ;

        /* Generate 9 clock pulses on SCL to attempt to clock out any data from a stuck slave */
        for (int i = 0; i < 9; i++) {
            I2C_PORT->ODR &= ~(1U << I2C_SCL_PIN); /* SCL low */
            for (volatile int j = 0; j < 100; j++)
                ; /* Clock low duration */
            I2C_PORT->ODR |= (1U << I2C_SCL_PIN); /* SCL high */
            for (volatile int j = 0; j < 100; j++)
                ; /* Clock high duration */
        }

        /* Generate a manual STOP condition: SCL high, SDA transitions low to high */
        I2C_PORT->ODR &= ~(1U << I2C_SDA_PIN); /* SDA low (while SCL is high) */
        for (volatile int j = 0; j < 100; j++)
            ;
        I2C_PORT->ODR |= (1U << I2C_SCL_PIN); /* SCL high */
        for (volatile int j = 0; j < 100; j++)
            ;
        I2C_PORT->ODR |= (1U << I2C_SDA_PIN); /* SDA high (generates STOP) */
        for (volatile int j = 0; j < 100; j++)
            ;

        /* Revert pins back to Alternate Function mode for I2C operation */
        I2C_PORT->MODER &= ~((3U << (I2C_SCL_PIN * 2))
                | (3U << (I2C_SDA_PIN * 2)));
        I2C_PORT->MODER |= (2U << (I2C_SCL_PIN * 2))
                | (2U << (I2C_SDA_PIN * 2));
    }

    /* 4. Reset I2C1 peripheral to clear any internal stuck state */
    I2C1->CR1 |= I2C_CR1_SWRST; /* Put I2C peripheral into reset state */
    I2C1->CR1 &= ~I2C_CR1_SWRST; /* Release I2C peripheral from reset state */

    /* 5. Configure I2C1 parameters */
    I2C1->CR1 &= ~I2C_CR1_PE; /* Disable peripheral (PE=0) before configuration */

    /* Set peripheral clock frequency (FREQ bits in CR2) */
    /* This must be configured with the APB1 clock frequency in MHz. */
    I2C1->CR2 = PCLK1_FREQ_MHZ;

    /* Configure CCR (Clock Control Register) for SCL frequency (Standard mode 100kHz) */
    I2C1->CCR = I2C_STANDARD_MODE_CCR;

    /* Configure TRISE (Rise Time Register) based on PCLK1 and max SCL rise time */
    I2C1->TRISE = I2C_TRISE_VALUE;

    I2C1->CR1 |= I2C_CR1_PE; /* Enable peripheral (PE=1) after configuration */
}

/**
 * @brief Generate I2C START condition.
 * This function sets the ACK bit (enabling acknowledgment for reception)
 * before generating the START condition. It then waits for the Start Bit (SB)
 * flag in SR1 to confirm the start condition has been sent.
 */
void I2C_Start(void) {
    /* Enable ACK before generating START. This is important for the master receiver mode. */
        /* For master transmitter, it doesn't harm. */
    I2C1->CR1 |= I2C_CR1_ACK;
    /* Generate START condition */
    I2C1->CR1 |= I2C_CR1_START;
    /* Wait for the START condition to be sent */
    while (!(I2C1->SR1 & I2C_SR1_SB));
}

/**
 * @brief Generate I2C stop condition
 * This function generates a STOP condition
 * and waits until the bus is not busy.
 * * It is good practice to ensure the bus is not busy
 * before generating a STOP condition,
 *
 */
void I2C_Stop(void) {
    /* Generate STOP condition by setting the STOP bit in CR1 */
    I2C1->CR1 |= I2C_CR1_STOP;
    /* Wait until BUSY flag is cleared */
    while (I2C1->SR2 & I2C_SR2_BUSY);
}

/**
 * @brief Send slave address with WRITE bit (LSB=0).
 * Waits for Address Acknowledged (ADDR) flag.
 * ADDR flag is cleared by reading SR1 then SR2.
 * @param slave_address The 7-bit I2C slave address.
 */
void I2C_addressWrite(uint8_t slave_address) {
    /* Temporary variable for reading status registers */
    volatile uint32_t temp;
    /* Send slave address shifted left by 1, with LSB=0 for write operation */
    I2C1->DR = (uint8_t) (slave_address << 1);
    /* Wait for ADDR (Address Acknowledged) flag to be set in SR1. */
    /* This indicates the slave has acknowledged its address. */
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    /* Clearing ADDR flag: This is done by a read to SR1 followed by a read to SR2. */
    /* The read of SR1 was done in the while loop condition. */
    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void) temp; /* Avoid unused variable warning if optimizations are high */
}

/**
 * @brief Send slave address with READ bit (LSB=1).
 * Waits for Address Acknowledged (ADDR) flag.
 * ADDR flag is cleared by reading SR1 then SR2.
 * @param slave_address The 7-bit I2C slave address.
 */
void I2C_addressRead(uint8_t slave_address) {
    /* Temporary variable for reading status registers */
    volatile uint32_t temp;
    /* Send slave address shifted left by 1, with LSB=1 for read operation */
    I2C1->DR = (uint8_t) ((slave_address << 1) | 1);
    /* Wait for ADDR (Address Acknowledged) flag to be set in SR1. */
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    /* Clearing ADDR flag: Read SR1 then SR2. */
    /* The read of SR1 was done in the while loop condition. */
    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void) temp; /* Avoid unused variable warning */
}

/**
 * @brief Write one byte of data to the slave.
 * Waits for Transmit buffer Empty (TXE) then Byte Transfer Finished (BTF).
 * @param byte The data byte to send.
 */
void I2C_writeByte(uint8_t byte) {
    /* Wait for TXE (Transmit data register empty) flag to be set in SR1. */
    /* This indicates that I2C_DR is empty and ready for the next byte. */
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    /* Write data to Data Register (DR) */
    I2C1->DR = byte;
    /* Wait for BTF (Byte Transfer Finished) flag to be set in SR1. */
    /* This indicates that the byte has been successfully transmitted (shifted out). */
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

/**
 * @brief Read one byte of data from the slave.
 * Manages ACK/NACK generation for multi-byte reads.
 * Waits for Receive buffer Not Empty (RXNE) flag.
 * @param ack Enum to control ACK/NACK:
 * MULTI_BYTE_ACK_ON: Send ACK (more bytes to read).
 * MULTI_BYTE_ACK_OFF: Send NACK (this is the last byte).
 * @return The received data byte.
 */
uint8_t I2C_readByte(ack_status_t ack) {
    if (ack == MULTI_BYTE_ACK_ON) {
        /* Enable ACK for the received byte (indicates more bytes to come) */
        I2C1->CR1 |= I2C_CR1_ACK;
    } else {
        /* Disable ACK (send NACK for the received byte - last byte) */
        I2C1->CR1 &= ~I2C_CR1_ACK;
    }

    /* Wait for RXNE (Receive data register not empty) flag to be set in SR1. */
    /* This indicates that a byte has been received and is ready to be read from I2C_DR. */
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    /* Read data from Data Register (DR) */
    return (uint8_t) I2C1->DR;
}
