#include "stm32f4xx.h"
#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "delay.h"
#include "i2c_driver.h"
#include "ds3231.h"
#include "lcd_parallel.h"
#include <stdio.h>
#include <string.h>

// --- PIN DEFINITIONS ---
// Buttons
#define BTN_UP_PORT     GPIOB
#define BTN_UP_PIN      12
#define BTN_DOWN_PORT   GPIOB
#define BTN_DOWN_PIN    13
#define BTN_LEFT_PORT   GPIOB
#define BTN_LEFT_PIN    14
#define BTN_RIGHT_PORT  GPIOB
#define BTN_RIGHT_PIN   15

// L298N Motor Control
#define FAN_PWM_PORT    GPIOA
#define FAN_PWM_PIN     8  // TIM1_CH1
#define FAN_IN1_PORT    GPIOA
#define FAN_IN1_PIN     2
#define FAN_IN2_PORT    GPIOA
#define FAN_IN2_PIN     3

// Heating LED
#define HEATER_PWM_PORT GPIOA
#define HEATER_PWM_PIN  9  // TIM1_CH2

// LM35 Sensor
#define LM35_ADC_PORT   GPIOA
#define LM35_ADC_PIN    0  // ADC1_IN0

// --- CONSTANTS ---
#define PWM_MAX_DUTY 1000 // For ARR = 999
#define TEMP_DIFF_MAX 10.0f // Max temp difference for 100% PWM

// --- TYPE DEFINITIONS ---
// UI Screen States
typedef enum {
    MAIN_SCREEN = 0,
    SLOT1_SCREEN,
    SLOT2_SCREEN,
    SLOT3_SCREEN,
    TIME_SCREEN,
    NUM_SCREENS // Total number of screens
} UI_State;

// Temperature Thresholds for each time slot
typedef struct {
    int8_t high_temp;
    int8_t low_temp;
} TempThreshold;

// --- GLOBAL VARIABLES ---
volatile float current_temp = 0.0f;
volatile uint8_t fan_status = 0; // 0: OFF, 1: ON
volatile uint8_t heater_status = 0; // 0: OFF, 1: ON
volatile uint16_t adc_val = 0; // ADC value from LM35

ds3231_time_t rtc_time;
TempThreshold thresholds[3];
UI_State current_screen = MAIN_SCREEN;
uint8_t editMode = 0; // 0: Normal, 1: Editing
uint8_t editCursor = 0; // 0: Edit T_H, 1: Edit T_L

// --- FUNCTION PROTOTYPES ---
void SystemClock_Config(void);
void Error_Handler(void);
void GPIO_Init_Pins(void);
void Timers_Init_PWM(void);
void Update_LCD(void);
void Handle_Buttons(void);
void Control_Temperature(void);
uint8_t readButtonDebounced(GPIO_TypeDef *GPIOx, uint8_t pin);

// --- MAIN FUNCTION ---
int main(void) {
    // --- INITIALIZATION ---
    HAL_Init(); // Initialize HAL Library
    SystemClock_Config(); // Configure system clock to 84MHz
    Delay_Init();
    GPIO_Init_Pins();
    I2C_Init();
    ADC_clockEnable(ADC1);
    ADC_Init(ADC1, ADC_Resolution_12BIT, ADC_ALIGN_RIGHT,
            ADC_CLK_PRESCALER_DIV4);
    ADC_configChannel(ADC1, 0, ADC_sampleTime_144CYCLES); // ADC1_IN0
    ADC_Enable(ADC1);
    Timers_Init_PWM();
    LCD_Init();

    // --- Set default thresholds ---
    // Slot 1: 07:00 - 17:00 (Work)
    thresholds[0].high_temp = 26;
    thresholds[0].low_temp = 23;
    // Slot 2: 17:00 - 22:00 (Living)
    thresholds[1].high_temp = 25;
    thresholds[1].low_temp = 22;
    // Slot 3: 22:00 - 07:00 (Sleep)
    thresholds[2].high_temp = 28;
    thresholds[2].low_temp = 25;

    // Optional: Set DS3231 time once
    //DS3231_setTime(21, 37, 00); // hh, mm, ss
    //DS3231_setDate(3, 17, 6, 25); // day_of_week, date, month, year
    LCD_Clear(); // Clear LCD before display
    Update_LCD(); // Initial display
    //LCD_Write("System Ready");

    // --- MAIN LOOP ---
    while (1) {
        Handle_Buttons();
        Control_Temperature();
        Update_LCD();
        delay_ms(100); // Main loop delay
    }
}

/**
 * @brief Initialize all GPIO pins for buttons and motor driver
 */
void GPIO_Init_Pins(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    // --- Buttons with Pull-up ---
    gpio_config_t btn_up_cfg = { BTN_UP_PORT, BTN_UP_PIN,
            GPIO_DRIVER_MODE_INPUT, 0, 0, GPIO_DRIVER_PULL_UP, 0 };
    GPIO_Init(&btn_up_cfg);
    gpio_config_t btn_down_cfg = { BTN_DOWN_PORT, BTN_DOWN_PIN,
            GPIO_DRIVER_MODE_INPUT, 0, 0, GPIO_DRIVER_PULL_UP, 0 };
    GPIO_Init(&btn_down_cfg);
    gpio_config_t btn_left_cfg = { BTN_LEFT_PORT, BTN_LEFT_PIN,
            GPIO_DRIVER_MODE_INPUT, 0, 0, GPIO_DRIVER_PULL_UP, 0 };
    GPIO_Init(&btn_left_cfg);
    gpio_config_t btn_right_cfg = { BTN_RIGHT_PORT, BTN_RIGHT_PIN,
            GPIO_DRIVER_MODE_INPUT, 0, 0, GPIO_DRIVER_PULL_UP, 0 };
    GPIO_Init(&btn_right_cfg);

    // --- L298N Direction Pins ---
    gpio_config_t fan_in1_cfg = { FAN_IN1_PORT, FAN_IN1_PIN,
            GPIO_DRIVER_MODE_OUTPUT, GPIO_DRIVER_OUTPUT_PUSH_PULL,
            GPIO_DRIVER_SPEED_LOW, GPIO_DRIVER_NO_PULL, 0 };
    GPIO_Init(&fan_in1_cfg);
    gpio_config_t fan_in2_cfg = { FAN_IN2_PORT, FAN_IN2_PIN,
            GPIO_DRIVER_MODE_OUTPUT, GPIO_DRIVER_OUTPUT_PUSH_PULL,
            GPIO_DRIVER_SPEED_LOW, GPIO_DRIVER_NO_PULL, 0 };
    GPIO_Init(&fan_in2_cfg);

    // --- GPIO for PWM Pins ---
    // PA8 for TIM1_CH1 (Fan)
    gpio_config_t fan_pwm_cfg = { FAN_PWM_PORT, FAN_PWM_PIN,
            GPIO_DRIVER_MODE_ALT_FUNCTION, GPIO_DRIVER_OUTPUT_PUSH_PULL,
            GPIO_DRIVER_SPEED_HIGH, GPIO_DRIVER_NO_PULL, GPIO_DRIVER_AF1 };
    GPIO_Init(&fan_pwm_cfg);
    // PA9 for TIM1_CH2 (Heater)
    gpio_config_t heater_pwm_cfg = { HEATER_PWM_PORT, HEATER_PWM_PIN,
            GPIO_DRIVER_MODE_ALT_FUNCTION, GPIO_DRIVER_OUTPUT_PUSH_PULL,
            GPIO_DRIVER_SPEED_HIGH, GPIO_DRIVER_NO_PULL, GPIO_DRIVER_AF1 };
    GPIO_Init(&heater_pwm_cfg);

    // --- ADC for LM35 ---
    gpio_config_t lm35 = { .port = LM35_ADC_PORT, .pin =
    LM35_ADC_PIN, .mode = GPIO_DRIVER_MODE_ANALOG, .otype =
            GPIO_DRIVER_OUTPUT_PUSH_PULL, .speed = GPIO_DRIVER_SPEED_LOW,
            .pull = GPIO_DRIVER_NO_PULL, };
    GPIO_Init(&lm35);

    // --- LCD Pins (Parallel Interface) ---
    /* LCD Pins PA5-PA10 - Output Push-Pull */
    gpio_config_t lcd_pins[] = { { GPIOB, 1, GPIO_DRIVER_MODE_OUTPUT,
            GPIO_DRIVER_OUTPUT_PUSH_PULL, GPIO_DRIVER_SPEED_MEDIUM,
            GPIO_DRIVER_NO_PULL }, { GPIOB, 0, GPIO_DRIVER_MODE_OUTPUT,
            GPIO_DRIVER_OUTPUT_PUSH_PULL, GPIO_DRIVER_SPEED_MEDIUM,
            GPIO_DRIVER_NO_PULL }, { GPIOA, 7, GPIO_DRIVER_MODE_OUTPUT,
            GPIO_DRIVER_OUTPUT_PUSH_PULL, GPIO_DRIVER_SPEED_MEDIUM,
            GPIO_DRIVER_NO_PULL }, { GPIOA, 6, GPIO_DRIVER_MODE_OUTPUT,
            GPIO_DRIVER_OUTPUT_PUSH_PULL, GPIO_DRIVER_SPEED_MEDIUM,
            GPIO_DRIVER_NO_PULL }, { GPIOB, 10, GPIO_DRIVER_MODE_OUTPUT,
            GPIO_DRIVER_OUTPUT_PUSH_PULL, GPIO_DRIVER_SPEED_MEDIUM,
            GPIO_DRIVER_NO_PULL }, { GPIOB, 2, GPIO_DRIVER_MODE_OUTPUT,
            GPIO_DRIVER_OUTPUT_PUSH_PULL, GPIO_DRIVER_SPEED_MEDIUM,
            GPIO_DRIVER_NO_PULL } };
    for (int i = 0; i < sizeof(lcd_pins) / sizeof(gpio_config_t); i++) {
        GPIO_Init(&lcd_pins[i]);
    }
}

/**
 * @brief Initialize TIM1 for PWM output on CH1 and CH2
 */
void Timers_Init_PWM(void) {
    // Enable TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Set Prescaler and Auto-Reload Register for a specific frequency (e.g., 1kHz)
    // F_pwm = F_clk / ((PSC+1) * (ARR+1)) = 84MHz / (84 * 1000) = 1kHz
    TIM1->PSC = 84 - 1;
    TIM1->ARR = PWM_MAX_DUTY - 1;

    // Configure Channel 1 (Fan)
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM mode 1
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload
    TIM1->CCER |= TIM_CCER_CC1E; // Enable Channel 1 output

    // Configure Channel 2 (Heater)
    TIM1->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // PWM mode 1
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE; // Enable preload
    TIM1->CCER |= TIM_CCER_CC2E; // Enable Channel 2 output

    TIM1->BDTR |= TIM_BDTR_MOE; // Main output enable for advanced timers

    TIM1->CCR1 = 0; // Initial duty cycle for Fan = 0
    TIM1->CCR2 = 0; // Initial duty cycle for Heater = 0

    // Enable Timer
    TIM1->CR1 |= TIM_CR1_CEN;
}

uint8_t readButtonDebounced(GPIO_TypeDef *GPIOx, uint8_t pin) {
    if (GPIO_Read(GPIOx, pin) == 0) {
        /* Wait for button to be released */
        delay_ms(50);
        /* Check if button is still pressed after delay */
        if (GPIO_Read(GPIOx, pin) == 0) {

            while (GPIO_Read(GPIOx, pin) == 0)
                ;

            return 1;
        }
    }
    return 0;
}

/**
 * @brief Updates the LCD screen based on the current UI state
 */
void Update_LCD(void) {
    static UI_State last_screen = -1;
    static uint8_t last_editMode = -1;
    static uint8_t last_editCursor = -1;
    char buffer1[20], buffer2[20];

    // Only update if state has changed to prevent flickering
    if (last_screen != current_screen || editMode) {
        LCD_Clear();
        last_screen = current_screen;
    }

    switch (current_screen) {
    case MAIN_SCREEN:
        sprintf(buffer1, "Temp: %.1f C", current_temp);
        sprintf(buffer2, "Fan:%-3s Heat:%-3s", fan_status ? "ON" : "OFF",
                heater_status ? "ON" : "OFF");
        break;

    case SLOT1_SCREEN:
        sprintf(buffer1, "Daytime: 07h-17h");
        if (editMode && editCursor == 0)
            sprintf(buffer2, ">TH:%-2d  TL:%-2d", thresholds[0].high_temp,
                    thresholds[0].low_temp);
        else if (editMode && editCursor == 1)
            sprintf(buffer2, " TH:%-2d >TL:%-2d", thresholds[0].high_temp,
                    thresholds[0].low_temp);
        else
            sprintf(buffer2, " T_H:%-2d  T_L:%-2d", thresholds[0].high_temp,
                    thresholds[0].low_temp);
        break;

    case SLOT2_SCREEN:
        sprintf(buffer1, "Evening: 17h-22h");
        if (editMode && editCursor == 0)
            sprintf(buffer2, ">TH:%-2d  TL:%-2d", thresholds[1].high_temp,
                    thresholds[1].low_temp);
        else if (editMode && editCursor == 1)
            sprintf(buffer2, " TH:%-2d >TL:%-2d", thresholds[1].high_temp,
                    thresholds[1].low_temp);
        else
            sprintf(buffer2, " T_H:%-2d  T_L:%-2d", thresholds[1].high_temp,
                    thresholds[1].low_temp);
        break;

    case SLOT3_SCREEN:
        sprintf(buffer1, "Night: 22h-06h");
        if (editMode && editCursor == 0)
            sprintf(buffer2, ">TH:%-2d  TL:%-2d", thresholds[2].high_temp,
                    thresholds[2].low_temp);
        else if (editMode && editCursor == 1)
            sprintf(buffer2, " TH:%-2d >TL:%-2d", thresholds[2].high_temp,
                    thresholds[2].low_temp);
        else
            sprintf(buffer2, " T_H:%-2d  T_L:%-2d", thresholds[2].high_temp,
                    thresholds[2].low_temp);
        break;

    case TIME_SCREEN:
        DS3231_getFullTime(&rtc_time);
        sprintf(buffer1, "Time: %02d:%02d:%02d", rtc_time.hours,
                rtc_time.minutes, rtc_time.seconds);
        sprintf(buffer2, "Date: %02d/%02d/20%02d", rtc_time.date,
                rtc_time.month, rtc_time.year);
        break;
    }

    LCD_setCursor(0, 0);
    LCD_Write(buffer1);
    LCD_setCursor(0, 1);
    LCD_Write(buffer2);
}

/**
 * @brief Handles button presses with debouncing
 */
void Handle_Buttons(void) {
    // Xử lý nút RIGHT
    if (readButtonDebounced(BTN_RIGHT_PORT, BTN_RIGHT_PIN)) {
        if (editMode) {
            editMode = 0; // Thoát chế độ chỉnh sửa
        } else {
            current_screen = (current_screen + 1) % NUM_SCREENS;
        }
    }

    // Xử lý nút LEFT
    if (readButtonDebounced(BTN_LEFT_PORT, BTN_LEFT_PIN)) {
        if (current_screen >= SLOT1_SCREEN && current_screen <= SLOT3_SCREEN) {
            if (!editMode) {
                editMode = 1;
                editCursor = 0;
            } else {
                editCursor = !editCursor;
            }
        }
    }

    // Xử lý nút UP
    if (readButtonDebounced(BTN_UP_PORT, BTN_UP_PIN)) {
        if (editMode) {
            uint8_t slot_idx = current_screen - SLOT1_SCREEN;
            if (editCursor == 0) { // Edit T_H
                thresholds[slot_idx].high_temp++;
            } else { // Edit T_L
                thresholds[slot_idx].low_temp++;
            }
            if (thresholds[slot_idx].low_temp
                    > thresholds[slot_idx].high_temp) {
                thresholds[slot_idx].low_temp = thresholds[slot_idx].high_temp;
            }
        }
    }

    // Xử lý nút DOWN
    if (readButtonDebounced(BTN_DOWN_PORT, BTN_DOWN_PIN)) {
        if (editMode) {
            uint8_t slot_idx = current_screen - SLOT1_SCREEN;
            if (editCursor == 0) { // Edit T_H
                thresholds[slot_idx].high_temp--;
            } else { // Edit T_L
                thresholds[slot_idx].low_temp--;
            }
            if (thresholds[slot_idx].high_temp
                    < thresholds[slot_idx].low_temp) {
                thresholds[slot_idx].high_temp = thresholds[slot_idx].low_temp;
            }
        }
    }
}

/**
 * @brief Reads temp and time, controls Fan and Heater PWM
 */
void Control_Temperature(void) {
    // 1. Read temperature from LM35
    ADC_startConversionSW(ADC1);
    while (!ADC_getFlagStatus(ADC1, ADC_SR_EOC))
        ;
    adc_val = ADC_readValue(ADC1);
    float voltage = adc_val * (3.3f / 4095.0f); // Assuming Vref = 3.3V, 12-bit ADC
    current_temp = voltage * 100.0f; // LM35 gives 10mV per degree Celsius

    // 2. Read time from DS3231 and determine current slot
    DS3231_getFullTime(&rtc_time);
    uint8_t current_slot_idx;
    if (rtc_time.hours >= 7 && rtc_time.hours < 17) {
        current_slot_idx = 0; // Slot 1
    } else if (rtc_time.hours >= 17 && rtc_time.hours < 22) {
        current_slot_idx = 1; // Slot 2
    } else {
        current_slot_idx = 2; // Slot 3 (22:00 - 06:59)
    }

    // 3. Get active thresholds
    int8_t t_high = thresholds[current_slot_idx].high_temp;
    int8_t t_low = thresholds[current_slot_idx].low_temp;

    // 4. Control Fan (Cooling)
    if (current_temp > t_high) {
        fan_status = 1;
        GPIO_Write(FAN_IN1_PORT, FAN_IN1_PIN, 1); // Set direction
        GPIO_Write(FAN_IN2_PORT, FAN_IN2_PIN, 0);

        float temp_diff = current_temp - t_high;
        uint16_t duty_cycle = (temp_diff / TEMP_DIFF_MAX) * PWM_MAX_DUTY;
        if (duty_cycle > PWM_MAX_DUTY) {
            duty_cycle = PWM_MAX_DUTY;
        }
        TIM1->CCR1 = duty_cycle;

    } else {
        fan_status = 0;
        TIM1->CCR1 = 0;
        GPIO_Write(FAN_IN1_PORT, FAN_IN1_PIN, 0); // Stop motor
        GPIO_Write(FAN_IN2_PORT, FAN_IN2_PIN, 0);
    }

    // 5. Control Heater (Heating)
    if (current_temp < t_low) {
        heater_status = 1;
        float temp_diff = t_low - current_temp;
        uint16_t duty_cycle = (temp_diff / TEMP_DIFF_MAX) * PWM_MAX_DUTY;
        if (duty_cycle > PWM_MAX_DUTY) {
            duty_cycle = PWM_MAX_DUTY;
        }
        TIM1->CCR2 = duty_cycle;
    } else {
        heater_status = 0;
        TIM1->CCR2 = 0;
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
