#include "gpio.h"

/**
 * @brief Initialize GPIO pin with specified configuration.
 *
 * @param cfg Pointer to GPIO configuration structure
 */
void GPIO_Init(const gpio_config_t *cfg)
{
    /* Enable clock for the corresponding GPIO port */
	uint32_t gpio_en_bit = ((uint32_t)cfg->port - GPIOA_BASE) / 0x400;
	RCC->AHB1ENR |= (1 << (gpio_en_bit + RCC_AHB1ENR_GPIOAEN_Pos));

    /* Configure pin mode */
    cfg->port->MODER &= ~(0x3 << (cfg->pin * 2));
    cfg->port->MODER |= ((cfg->mode & 0x3) << (cfg->pin * 2));

    /* Configure output type */
    cfg->port->OTYPER &= ~(1 << cfg->pin);
    cfg->port->OTYPER |= (cfg->otype << cfg->pin);

    /* Configure output speed */
    cfg->port->OSPEEDR &= ~(0x3 << (cfg->pin * 2));
    cfg->port->OSPEEDR |= (cfg->speed << (cfg->pin * 2));

    /* Configure pull-up / pull-down resistors */
    cfg->port->PUPDR &= ~(0x3 << (cfg->pin * 2));
    cfg->port->PUPDR |= (cfg->pull << (cfg->pin * 2));

    /* Configure alternate function if needed */
    if (cfg->mode == GPIO_DRIVER_MODE_ALT_FUNCTION)
    {
    	GPIO_SetAlternateFunction(cfg->port, cfg->pin, cfg->af);
    }
}

/**
 * @brief Set alternate function for a GPIO pin.
 *
 * @param port GPIO port (GPIOA~GPIOH).
 * @param pin  GPIO pin number (0~15).
 * @param af   Alternate function selection (AF0~AF15).
 */
void GPIO_SetAlternateFunction(GPIO_TypeDef *port, uint8_t pin, gpio_alt_function_t af)
{
	volatile uint32_t *afr = &port->AFR[pin >> 3]; /* AFR[0] or AFR[1] */
	*afr = (*afr & ~(0xF << ((pin & 0x7) * 4))) | (af << ((pin & 0x7) * 4));
}

/**
 * @brief Write a digital value to a GPIO pin.
 *
 * @param port GPIO port.
 * @param pin  GPIO pin number.
 * @param value 0 for low, non-zero for high.
 */
void GPIO_Write(GPIO_TypeDef *port, uint8_t pin, uint8_t value)
{
	port->BSRR = (1 << (pin + (value ? 0 : 16)));
}

/**
 * @brief Read digital input value from a GPIO pin.
 *
 * @param port GPIO port.
 * @param pin  GPIO pin number.
 * @return uint8_t 1 if high, 0 if low.
 */
uint8_t GPIO_Read(GPIO_TypeDef *port, uint8_t pin)
{
    return (port->IDR >> pin) & 0x01;
}

/**
 * @brief Toggle the current output state of a GPIO pin.
 *
 * @param port GPIO port.
 * @param pin  GPIO pin number.
 */
void GPIO_Toggle(GPIO_TypeDef *port, uint8_t pin)
{
    port->ODR ^= (1 << pin);
}
