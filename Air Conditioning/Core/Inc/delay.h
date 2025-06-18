#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f4xx.h"
#include <stdio.h>

/**
 * @brief Initializes the SysTick for millisecond delays and the DWT for microsecond delays.
 * @note  This function must be called once at the beginning of the main function,
 * after the system clock has been configured. The SystemCoreClock global
 * variable must be up-to-date before calling this function.
 */
void Delay_Init(void);

/**
 * @brief Provides a blocking delay in microseconds.
 * @note  This function uses the DWT cycle counter for high accuracy. It is
 * suitable for short delays required by peripheral drivers.
 * @param us: The number of microseconds to delay.
 */
void delay_us(uint32_t us);

/**
 * @brief Provides a blocking delay in milliseconds.
 * @note  This function relies on the SysTick interrupt.
 * @param ms: The number of milliseconds to delay.
 */
void delay_ms(uint32_t ms);


#endif /* DELAY_H_ */
