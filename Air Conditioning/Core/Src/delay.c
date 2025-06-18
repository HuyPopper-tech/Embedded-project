#include "delay.h"

/* Tick counter for millisecond delays */
static volatile uint32_t systick_ms_count = 0;

/**
 * @brief Initializes the SysTick for millisecond delays and the DWT for microsecond delays.
 * @note  This function must be called once at the beginning of the main function,
 * after the system clock has been configured. The SystemCoreClock global
 * variable must be up-to-date before calling this function.
 */
void Delay_Init(void) {
    /* 1. Configure and enable SysTick for a 1ms interrupt (for Delay_ms) */
    if (SysTick_Config(SystemCoreClock / 1000) != 0) {
        /* Capture error if SysTick configuration fails */
        while (1);
    }

    /* 2. Enable the DWT Cycle Counter (for Delay_us) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Enable Trace & Debug block */
    DWT->CYCCNT = 0; /* Reset the cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Enable the cycle counter */
}

/**
 * @brief This function handles the System tick timer interrupt.
 * It increments the millisecond tick counter.
 * @note This function is called by the SysTick interrupt handler.
 */
void SysTick_Handler(void) {
    systick_ms_count++;
}

/**
 * @brief Provides a blocking delay in microseconds.
 * @note  This function uses the DWT cycle counter for high accuracy. It is
 * suitable for short delays required by peripheral drivers.
 * @param us: The number of microseconds to delay.
 */
void delay_us(uint32_t us) {
    /* Get the number of CPU cycles per microsecond */
    uint32_t ticks_per_us = SystemCoreClock / 1000000;

    /* Get the current value of the cycle counter */
    uint32_t start_ticks = DWT->CYCCNT;

    /* Calculate the number of cycles to wait for */
    uint32_t ticks_to_wait = us * ticks_per_us;

    /* Wait until the cycle counter has increased by the required amount */
    while ((DWT->CYCCNT - start_ticks) < ticks_to_wait);
}

/**
 * @brief Provides a blocking delay in milliseconds.
 * @note  This function relies on the SysTick interrupt.
 * @param ms: The number of milliseconds to delay.
 */
void delay_ms(uint32_t ms) {
    /* Check if the delay is zero */
    if (ms == 0) {
        return;
    }
    uint32_t tickstart = systick_ms_count;
    /* Wait until the specified number of milliseconds has passed */
    while ((systick_ms_count - tickstart) < ms)
        ;
}
