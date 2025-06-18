#ifndef GPIO_H
#define GPIO_H

#include "stm32f4xx.h"

/**
 * @brief GPIO pin mode options
 */
typedef enum {
    GPIO_DRIVER_MODE_INPUT        = 0x00, /**< Input mode (floating) */
    GPIO_DRIVER_MODE_OUTPUT       = 0x01, /**< Output mode */
    GPIO_DRIVER_MODE_ALT_FUNCTION = 0x02, /**< Alternate function mode */
    GPIO_DRIVER_MODE_ANALOG       = 0x03  /**< Analog mode */
} gpio_mode_t;

/**
 * @brief GPIO output type options
 */
typedef enum {
    GPIO_DRIVER_OUTPUT_PUSH_PULL  = 0x00, /**< Push-pull output */
    GPIO_DRIVER_OUTPUT_OPEN_DRAIN = 0x01  /**< Open-drain output */
} gpio_output_type_t;

/**
 * @brief GPIO pull-up/pull-down configuration
 */
typedef enum {
    GPIO_DRIVER_NO_PULL   = 0x00, /**< No pull-up or pull-down */
    GPIO_DRIVER_PULL_UP   = 0x01, /**< Pull-up configuration */
    GPIO_DRIVER_PULL_DOWN = 0x02  /**< Pull-down configuration */
} gpio_pull_t;

/**
 * @brief GPIO speed configuration
 */
typedef enum {
    GPIO_DRIVER_SPEED_LOW    = 0x00, /**< Low speed (2 MHz) */
    GPIO_DRIVER_SPEED_MEDIUM = 0x01, /**< Medium speed (25 MHz) */
    GPIO_DRIVER_SPEED_FAST   = 0x02, /**< Fast speed (50 MHz) */
    GPIO_DRIVER_SPEED_HIGH   = 0x03  /**< High speed (100 MHz) */
} gpio_speed_t;

/**
 * @brief GPIO alternate function configuration
 */
typedef enum {
    GPIO_DRIVER_AF0 = 0x00, GPIO_DRIVER_AF1,  GPIO_DRIVER_AF2,  GPIO_DRIVER_AF3,
    GPIO_DRIVER_AF4,        GPIO_DRIVER_AF5,  GPIO_DRIVER_AF6,  GPIO_DRIVER_AF7,
    GPIO_DRIVER_AF8,        GPIO_DRIVER_AF9,  GPIO_DRIVER_AF10, GPIO_DRIVER_AF11,
    GPIO_DRIVER_AF12,       GPIO_DRIVER_AF13, GPIO_DRIVER_AF14, GPIO_DRIVER_AF15
} gpio_alt_function_t;

/**
 * @brief GPIO pin configuration structure
 */
typedef struct {
    GPIO_TypeDef *port;        /**< GPIO port (GPIOA, GPIOB, etc.) */
    uint8_t pin;               /**< Pin number (0-15) */
    gpio_mode_t mode;          /**< Mode: input, output, AF, analog */
    gpio_output_type_t otype;  /**< Output type: push-pull or open-drain */
    gpio_speed_t speed;        /**< Output speed */
    gpio_pull_t pull;          /**< Pull-up/pull-down config */
    gpio_alt_function_t af;    /**< Alternate function if used */
} gpio_config_t;

/**
 * @brief Initialize a GPIO pin with the given configuration
 * @param cfg Pointer to GPIO configuration struct
 */
void GPIO_Init(const gpio_config_t *cfg);

/**
 * @brief Write logic value to GPIO pin
 * @param port GPIO port
 * @param pin Pin number
 * @param value 0 = Low, non-zero = High
 */
void GPIO_Write(GPIO_TypeDef *port, uint8_t pin, uint8_t value);

/**
 * @brief Read logic level of GPIO pin
 * @param port GPIO port
 * @param pin Pin number
 * @return 0 = Low, 1 = High
 */
uint8_t GPIO_Read(GPIO_TypeDef *port, uint8_t pin);

/**
 * @brief Toggle output level of GPIO pin
 * @param port GPIO port
 * @param pin Pin number
 */
void GPIO_Toggle(GPIO_TypeDef *port, uint8_t pin);

/**
 * @brief Set alternate function for a GPIO pin
 * @param port GPIO port
 * @param pin Pin number
 * @param af Alternate function selection
 */
void GPIO_SetAlternateFunction(GPIO_TypeDef *port, uint8_t pin, gpio_alt_function_t af);

#endif  /* GPIO_H */
