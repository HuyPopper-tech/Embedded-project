#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/* ADC Resolution (CR1 Register, RES bits) */
typedef enum {
    /* 00: 12-bit (15 ADCCLK cycles) */
    ADC_Resolution_12BIT = 0x00,
    /* 01: 10-bit (13 ADCCLK cycles) */
    ADC_Resolution_10BIT = 0x01,
    /* 10: 8-bit (11 ADCCLK cycles) */
    ADC_Resolution_8BIT = 0x02,
    /* 11: 6-bit (9 ADCCLK cycles) */
    ADC_Resolution_6BIT = 0x03
} ADC_Resolution;

/* ADC Data Alignment (CR2 Register, ALIGN bit) */
typedef enum {
    /* 0: Right alignment */
    ADC_ALIGN_RIGHT = 0x00,
    /* 1: Left alignment */
    ADC_ALIGN_LEFT = 0x01
} ADC_dataAlign;

/* ADC Common Prescaler (CCR Register, ADCPRE bits) - Affects all ADCs */
typedef enum {
    ADC_CLK_PRESCALER_DIV2 = 0x00, /* 00: PCLK2 divided by 2 */
    ADC_CLK_PRESCALER_DIV4 = 0x01, /* 01: PCLK2 divided by 4 */
    ADC_CLK_PRESCALER_DIV6 = 0x02, /* 10: PCLK2 divided by 6 */
    ADC_CLK_PRESCALER_DIV8 = 0x03 /* 11: PCLK2 divided by 8 */
} ADC_clockPrescaler;

/*
 * ADC Sampling Time (SMPR1/SMPR2 Registers, SMPx bits)
 * These values correspond to the bit patterns for the SMPx[2:0] bits
 */
typedef enum {
    ADC_sampleTime_3CYCLES = 0x00, /* 000: 3 cycles */
    ADC_sampleTime_15CYCLES = 0x01, /* 001: 15 cycles */
    ADC_sampleTime_28CYCLES = 0x02, /* 010: 28 cycles*/
    ADC_sampleTime_56CYCLES = 0x03, /* 011: 56 cycles */
    ADC_sampleTime_84CYCLES = 0x04, /* 100: 84 cycles*/
    ADC_sampleTime_112CYCLES = 0x05, /* 101: 112 cycles */
    ADC_sampleTime_144CYCLES = 0x06, /* 110: 144 cycles */
    ADC_sampleTime_480CYCLES = 0x07 /* 111: 480 cycles */
} ADC_sampleTime;

/**
 * @brief ADC DMA Mode
 * This enum defines the DMA modes for ADC transfers.
 * Normal mode transfers data once per conversion,
 * while Circular mode allows continuous transfer
 */
typedef enum {
    ADC_DMA_MODE_NORMAL = 0x00, /* Normal DMA mode */
    ADC_DMA_MODE_CIRCULAR = 0x01 /* Circular DMA mode */
} ADC_DMA_Mode;

/**
 * @brief ADC DMA Flags
 * These flags represent the DMA status for ADC transfers.
 * They can be used to check the status of the DMA transfer.
 */
typedef enum {
    ADC_DMA_FLAG_TC, /* Transfer Complete */
    ADC_DMA_FLAG_HT, /* Half Transfer */
    ADC_DMA_FLAG_TE, /* Transfer Error */
    ADC_DMA_FLAG_DME, /* Direct Mode Error */
    ADC_DMA_FLAG_FE /* FIFO Error */
} ADC_DMA_genericFlag;

/**
 * @brief Enables the clock for the specified ADC peripheral.
 * @note This function assumes the ADC peripheral is already enabled in RCC.
 */
void ADC_clockEnable(ADC_TypeDef *adc);

/**
 * @brief Disables the clock for the specified ADC peripheral.
 */
void ADC_clockDisable(ADC_TypeDef *adc);

/**
 * @brief Initializes the ADC peripheral.
 * @note ADC common prescaler (ADCPRE) is set here and affects all ADCs.
 */
void ADC_Init(ADC_TypeDef *adc, ADC_Resolution resolution,
        ADC_dataAlign alignment, ADC_clockPrescaler prescaler);

/**
 * @brief De-initializes the specified ADC peripheral.
 */
void ADC_deInit(ADC_TypeDef *adc);

/**
 * @brief Configures a single channel for regular conversion sequence.
 */
void ADC_configChannel(ADC_TypeDef *adc, uint8_t channel,
        ADC_sampleTime sampleTime);

/**
 * @brief Enables the specified ADC peripheral.
 */
void ADC_Enable(ADC_TypeDef *adc);

/**
 * @brief Disables the specified ADC peripheral.
 */
void ADC_Disable(ADC_TypeDef *adc);

/**
 * @brief Starts a single ADC conversion using software trigger for the specified ADC.
 */
void ADC_startConversionSW(ADC_TypeDef *adc);

/**
 * @brief Polls the End Of Conversion (EOC) flag for the specified ADC.
 */
uint8_t ADC_pollForConversion(ADC_TypeDef *adc, volatile uint32_t timeout);

/**
 * @brief Reads the converted data from the specified ADC data register.
 */
uint16_t ADC_readValue(ADC_TypeDef *adc);

/**
 * @brief Enables the End Of Conversion (EOC) interrupt.
 * @note You MUST also configure and enable the corresponding ADC interrupt line
 * in the NVIC (Nested Vectored Interrupt Controller) separately.
 */
void ADC_enableEOCInterrupt(ADC_TypeDef *adc);

/**
 * @brief Disables the End Of Conversion (EOC) interrupt.
 */
void ADC_disableEOCInterrupt(ADC_TypeDef *adc);

/**
 * @brief Checks if a specific ADC status flag is set for the specified ADC.
 */
uint8_t ADC_getFlagStatus(ADC_TypeDef *adc, uint32_t flag);

/**
 * @brief Clears a specific ADC status flag for the specified ADC.
 */
void ADC_clearFlag(ADC_TypeDef *adc, uint32_t flag);

/**
 * @brief  Configures ADC and DMA for data transfer for the specified ADC.
 */
void ADC_DMA_Config(ADC_TypeDef *adc, uint16_t *pData, uint16_t length,
        ADC_DMA_Mode dmaMode);

/**
 * @brief Enables the DMA stream configured for the specified ADC.
 */
void ADC_DMA_Enable(ADC_TypeDef *adc);

/**
 * @brief Disables the DMA stream configured for the specified ADC.
 */
void ADC_DMA_Disable(ADC_TypeDef *adc);

/**
 * @brief Gets the status of a specific DMA flag for the specified ADC.
 */
uint8_t ADC_DMA_getFlagStatus(ADC_TypeDef *adc,
        ADC_DMA_genericFlag generic_flag);

/**
 * @brief Clears a specific DMA stream generic flag for the DMA associated with the specified ADC.
 */
void ADC_DMA_clearFlag(ADC_TypeDef *adc, ADC_DMA_genericFlag generic_flag);

#ifdef __cplusplus
}
#endif

#endif // __ADC_H
