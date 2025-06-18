#include "adc.h"
#include "stm32f4xx.h"
#include <stdbool.h>

#define ADC_CR1_RES_Pos             (24U) /* Position of RES bits in CR1 */
#define ADC_CCR_ADCPRE_Pos          (16U) /* Position of ADCPRE bits in CCR */
#define ADC_SMPR_SMP_CLEAR_Mask     (0x07UL) /* Mask to clear SMP bits (3 bits) */
#define ADC_SQR3_SQ1_CLEAR_Mask     (0x1FUL) /* Mask to clear SQ1 bits (5 bits) */

typedef struct {
    DMA_TypeDef        *DMAx; /* DMA peripheral */
    DMA_Stream_TypeDef *DMA_Stream; /* DMA stream */
    uint32_t           DMA_Channel; /* DMA channel */
    uint8_t            DMA_streamNumber; /* Stream number */
    uint32_t           RCC_AHB1ENR_DMAxEN; /* RCC AHB1ENR bit for DMA */
} ADC_DMA_configInfo;

/**
 * @brief Simple delay function (busy-wait loop).
 */
static void simple_delay(volatile uint32_t count) {
    while (count--)
        ;
}

/**
 * @brief Configures the DMA for ADC operations.
 * @param adc Pointer to the ADC peripheral.
 */
static bool ADC_DMA_getInfo(ADC_TypeDef *adc, ADC_DMA_configInfo *info) {
    if (adc == ADC1) {
        info->DMAx = DMA2;
        info->DMA_Stream = DMA2_Stream0;
        info->DMA_Channel = (0UL << DMA_SxCR_CHSEL_Pos);
        info->DMA_streamNumber = 0;
        info->RCC_AHB1ENR_DMAxEN = RCC_AHB1ENR_DMA2EN;
        return true;
    }
#ifdef ADC2
    else if (adc == ADC2) {
        info->DMAx = DMA2;
        info->DMA_Stream = DMA2_Stream2;
        info->DMA_Channel = (1UL << DMA_SxCR_CHSEL_Pos);
        info->DMA_streamNumber = 2;
        info->RCC_AHB1ENR_DMAxEN = RCC_AHB1ENR_DMA2EN;
        return true;
    }
#endif

#ifdef ADC3
    else if (adc == ADC3) {
        info->DMAx = DMA2;
        info->DMA_Stream = DMA2_Stream1;
        info->DMA_Channel = (2UL << DMA_SxCR_CHSEL_Pos);
        info->DMA_streamNumber = 1;
        info->RCC_AHB1ENR_DMAxEN = RCC_AHB1ENR_DMA2EN;
        return true;
    }
#endif
    return false;
}

/**
 * @brief Gets the specific DMA flag for the ADC.
 * @param generic_flag The generic DMA flag to check.
 * @param stream_number The stream number (0-7).
 * @return The specific DMA flag bit mask.
 */
static uint32_t ADC_DMA_getSpecificFlag(ADC_DMA_genericFlag generic_flag,
        uint8_t stream_number) {
    uint32_t bit_pos_in_group = 0;
    switch (generic_flag) {
    case ADC_DMA_FLAG_FE:
        bit_pos_in_group = 0;
        break;
    case ADC_DMA_FLAG_DME:
        bit_pos_in_group = 2;
        break;
    case ADC_DMA_FLAG_TE:
        bit_pos_in_group = 3;
        break;
    case ADC_DMA_FLAG_HT:
        bit_pos_in_group = 4;
        break;
    case ADC_DMA_FLAG_TC:
        bit_pos_in_group = 5;
        break;
    default:
        return 0;
    }

    uint8_t group_offset_factor;
    if (stream_number <= 3) {
        group_offset_factor = stream_number; // Lower group (0-3)
    } else {
        group_offset_factor = stream_number - 4; // Upper group (4-7)
    }
    return (1UL << (bit_pos_in_group + group_offset_factor * 6));
}

/**
 * @brief Enables the clock for the selected ADC peripheral.
 * @note This function assumes the ADC peripheral is already enabled in RCC.
 * @param adc Pointer to the ADC peripheral.
 */
void ADC_clockEnable(ADC_TypeDef *adc) {
    if (adc == ADC1) {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    }
    /* Check if ADC2 or ADC3 is defined before enabling their clocks */
#ifdef ADC2
    else if (adc == ADC2) {
        RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
    }
#endif

#ifdef ADC3
    else if (adc == ADC3) {
        RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    }
    #endif
    /* Ensure the write is completed */
    (void) RCC->APB2ENR;
}

/**
 * @brief Disables the clock for the selected ADC peripheral.
 * @param adc Pointer to the ADC peripheral.
 */
void ADC_clockDisable(ADC_TypeDef *adc) {
    if (adc == ADC1) {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    }
#ifdef ADC2
    else if (adc == ADC2) {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
    }
#endif

#ifdef ADC3
    else if (adc == ADC3) {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
    }
#endif
}

/**
 * @brief Initializes the ADC peripheral with specified parameters.
 * @param adc Pointer to the ADC peripheral.
 * @param resolution ADC resolution (12-bit, 10-bit, etc.).
 * @param alignment Data alignment (left or right).
 * @param prescaler ADC clock prescaler.
 * This function configures the ADC registers according to the specified parameters.
 */
void ADC_Init(ADC_TypeDef *adc, ADC_Resolution resolution,
        ADC_dataAlign alignment, ADC_clockPrescaler prescaler) {
    /* Validate parameters */
    if (!adc) {
        return;
    }

    /* Ensure ADC is disabled before configuration */
    if (adc->CR2 & ADC_CR2_ADON) {
        adc->CR2 &= ~ADC_CR2_ADON;
        simple_delay(100);
    }

    /* 1. Configure Common ADC settings (CCR) */
    /* Reset prescaler bits */
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    /* Set new prescaler */
    ADC->CCR |= (uint32_t) prescaler << ADC_CCR_ADCPRE_Pos;
    /* Configure other CCR settings if needed (e.g., DMA, Multi-mode, Vbat/Temp sensor - default off) */
    ADC->CCR &= ~(ADC_CCR_MULTI | ADC_CCR_DMA |
    ADC_CCR_DDS | ADC_CCR_DELAY | ADC_CCR_TSVREFE | ADC_CCR_VBATE);

    /* 2. Configure ADC Control Register 1 (CR1) */
    /* Reset relevant bits: Clear RES, SCAN, DISCEN, OVRIE, EOCIE */
    adc->CR1 &= ~(ADC_CR1_RES | ADC_CR1_SCAN | ADC_CR1_DISCEN | ADC_CR1_OVRIE
            | ADC_CR1_EOCIE);
    /* Set resolution */
    adc->CR1 |= ((uint32_t) resolution << ADC_CR1_RES_Pos);

    /* 3. Configure ADC Control Register 2 (CR2) */
    /* Reset relevant bits */
    adc->CR2 &= ~(ADC_CR2_ALIGN | ADC_CR2_CONT | ADC_CR2_DMA |
    ADC_CR2_DDS | ADC_CR2_EOCS | ADC_CR2_EXTEN | ADC_CR2_EXTSEL
            | ADC_CR2_SWSTART);
    /* Set data alignment */
    if (alignment == ADC_ALIGN_LEFT) {
        adc->CR2 |= ADC_CR2_ALIGN;
    }

    /* EOCS = 0: EOC flag is set at the end of single channel conversion (needed for basic polling/IT) */
    adc->CR2 &= ~ADC_CR2_EOCS;
    /* CONT = 0: Single conversion mode */
    adc->CR2 &= ~ADC_CR2_CONT;

    /* 4. Configure Sequence Length (SQR1) */
    /* Set sequence length to 1 (for single channel conversion) */
    adc->SQR1 &= ~ADC_SQR1_L;

    /* 5. Clear Status Register */
    adc->SR = 0;
}

/**
 * @brief De-initializes the ADC peripheral.
 * This function resets all ADC registers to their default values.
 * @param adc Pointer to the ADC peripheral.
 */
void ADC_deInit(ADC_TypeDef *adc) {
    if (!adc) {
        return;
    }

    /* Disable ADC first */
    if (adc->CR2 & ADC_CR2_ADON) {
        ADC_Disable(adc);
    }

    /*--- Reset ADC Registers (Consult RM for specific reset values if needed, often 0) ---*/
    adc->CR1 = 0x00000000;
    adc->CR2 = 0x00000000;
    adc->SMPR1 = 0x00000000;
    adc->SMPR2 = 0x00000000;
    adc->JOFR1 = 0x00000000;
    adc->JOFR2 = 0x00000000;
    adc->JOFR3 = 0x00000000;
    adc->JOFR4 = 0x00000000;
    adc->HTR = 0x00000FFF;
    adc->LTR = 0x00000000;
    adc->SQR1 = 0x00000000;
    adc->SQR2 = 0x00000000;
    adc->SQR3 = 0x00000000;
    adc->JSQR = 0x00000000;
    adc->SR = 0x00000000;
}

/**
 * @brief Configures a single ADC channel for regular conversion sequence.
 * @param adc Pointer to the ADC peripheral.
 * @param channel ADC channel number (0-18).
 * @param sampleTime Sampling time for the channel.
 * This function sets the sampling time and the sequence rank for the specified channel.
 */
void ADC_configChannel(ADC_TypeDef *adc, uint8_t channel,
        ADC_sampleTime sampleTime) {
    if (!adc || channel > 18 || sampleTime > ADC_sampleTime_480CYCLES) {
        return; // Invalid parameters
    }
    /* 0 for SMPR2, 1 for SMPR1 */
    uint32_t smpr_index;
    uint32_t smp_bit_offset;

    /* 1. Configure Sampling Time ---
     * Determine which SMPR register (SMPR1 for channels 10-18, SMPR2 for 0-9)
     * and the bit offset within the register (3 bits per channel)
     */
    if (channel < 10) {
        smpr_index = 2; // Use SMPR2
        smp_bit_offset = (channel % 10) * 3;
        /* Clear the 3 bits for the channel */
        adc->SMPR2 &= ~(ADC_SMPR_SMP_CLEAR_Mask << smp_bit_offset);
        /* Set the new sampling time */
        adc->SMPR2 |= ((uint32_t) sampleTime << smp_bit_offset);
    } else {
        smpr_index = 1; // Use SMPR1
        smp_bit_offset = (channel % 10) * 3;
        /* Clear the 3 bits for the channel */
        adc->SMPR1 &= ~(ADC_SMPR_SMP_CLEAR_Mask << smp_bit_offset);
        /* Set the new sampling time */
        adc->SMPR1 |= ((uint32_t) sampleTime << smp_bit_offset);
    }

    /* 2. Configure Regular Sequence (Rank 1)
     * Assuming sequence length is 1 (set in Init)
     * Set the first conversion in the sequence (SQ1) in SQR3 register */
    adc->SQR3 &= ~ADC_SQR3_SQ1;
    /* Set channel number (0-18) */
    adc->SQR3 |= (channel & ADC_SQR3_SQ1_CLEAR_Mask);
}

/**
 * @brief Enables the ADC peripheral (sets the ADON bit).
 * @note This function waits for the ADC to stabilize after enabling.
 * @param adc Pointer to the ADC peripheral.
 * This function sets the ADON bit in the ADC control register to enable the ADC.
 */
void ADC_Enable(ADC_TypeDef *adc) {
    if (!adc) {
        return;
    }
    if (!(adc->CR2 & ADC_CR2_ADON)) {
        adc->CR2 |= ADC_CR2_ADON;
        simple_delay(100);
    }
}

/**
 * @brief Disables the ADC peripheral (clears the ADON bit).
 * @note This function waits for the ADC to be completely disabled.
 * @param adc Pointer to the ADC peripheral.
 */
void ADC_Disable(ADC_TypeDef *adc) {
    if (!adc) {
        return;
    }
    /* Ensure no conversion is ongoing before disabling */
    if (adc->CR2 & ADC_CR2_ADON) {
        adc->CR2 &= ~ADC_CR2_ADON;
        simple_delay(10);
    }
}

/**
 * @brief Starts a single ADC conversion using software trigger.
 * @param adc Pointer to the ADC peripheral.
 * This function sets the SWSTART bit in the ADC control register to initiate a conversion.
 */
void ADC_startConversionSW(ADC_TypeDef *adc) {
    if (!adc) {
        return; // Null pointer check
    }

    /* EOCS=0: EOC flag set at end of conversion */
    adc->CR2 &= ~ADC_CR2_EOCS;

    /* Clear EOC flag before starting conversion */
    adc->SR &= ~ADC_SR_EOC;

    /* Set SWSTART bit to begin conversion */
    adc->CR2 |= ADC_CR2_SWSTART;
}

/**
 * @brief Polls the End Of Conversion (EOC) flag for the specified ADC.
 * @param adc Pointer to the ADC peripheral.
 * @param timeout Timeout in milliseconds for polling.
 * @return 1 if conversion completed successfully, 0 if timed out.
 * This function waits for the EOC flag to be set, indicating that the conversion is complete.
 */
uint8_t ADC_pollForConversion(ADC_TypeDef *adc, volatile uint32_t timeout) {
    if (!adc) {
        return 0;
    }
    uint8_t timed_out = 0;

    if (timeout == 0) {
        /* Infinite polling (blocking mode) */
        while (!(adc->SR & ADC_SR_EOC)) {
        }
    } else {
        /* Wait with timeout */
        while (!(adc->SR & ADC_SR_EOC)) {
            if (--timeout == 0) {
                timed_out = 1;
                break;
            }
        }
    }

    /* Return 1 if EOC is set, 0 if timed out */
    return (timed_out == 0);
}

/**
 * @brief Reads the converted data from the ADC data register.
 * @param adc Pointer to the ADC peripheral.
 * @return The converted value (12-bit resolution).
 */
uint16_t ADC_readValue(ADC_TypeDef *adc) {
    if (!adc) {
        return 0;
    }
    /* Reading DR clears the EOC flag in single conversion mode when EOCS=0 */
    return (uint16_t) (adc->DR);
}

/**
 * @brief Enables the End Of Conversion (EOC) interrupt.
 * @param adc Pointer to the ADC peripheral.
 * This function sets the EOCIE bit in the ADC control register
 */
void ADC_enableEOCInterrupt(ADC_TypeDef *adc) {
    if (!adc) {
        return;
    }
    adc->CR1 |= ADC_CR1_EOCIE;
    /* Enable NVIC interrupt for ADC */
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1);
}

/**
 * @brief Disables the End Of Conversion (EOC) interrupt.
 * @param adc Pointer to the ADC peripheral.
 * This function clears the EOCIE bit in the ADC control register
 */
void ADC_disableEOCInterrupt(ADC_TypeDef *adc) {
    if (!adc) {
        return;
    }
    /* Disable EOC interrupt in ADC CR1 */
    adc->CR1 &= ~ADC_CR1_EOCIE;
    /* Disable NVIC interrupt for ADC */
    NVIC_DisableIRQ(ADC_IRQn);
}

/**
 * @brief Checks if a specific ADC status flag is set.
 * @param adc Pointer to the ADC peripheral.
 * @param flag ADC status flag to check (e.g., ADC_SR_EOC).
 * @return 1 if the flag is set, 0 if not.
 */
uint8_t ADC_getFlagStatus(ADC_TypeDef *adc, uint32_t flag) {
    if (!adc) {
        return 0;
    }
    return ((adc->SR & flag) != 0);
}

/**
 * @brief Clears a specific ADC status flag.
 * @param adc Pointer to the ADC peripheral.
 * @param flag ADC status flag to clear (e.g., ADC_SR_EOC).
 * This function clears the specified flag by writing 0 to it.
 */
void ADC_clearFlag(ADC_TypeDef *adc, uint32_t flag) {
    if (!adc) {
        return;
    }

    /* Standard way to clear flags by writing 0 */
    adc->SR &= ~flag;
}

/**
 * @brief Configures the DMA for ADC operations.
 * @param adc Pointer to the ADC peripheral.
 * @param pData Pointer to the memory where ADC data will be stored.
 * @param length Number of data items to transfer.
 * @param dmaMode DMA mode (normal or circular).
 * This function sets up the DMA stream for transferring ADC data to memory.
 * It configures the DMA stream, sets the peripheral and memory addresses,
 */
void ADC_DMA_Config(ADC_TypeDef *adc, uint16_t *pData, uint16_t length,
        ADC_DMA_Mode dmaMode) {
    if (!adc || !pData || length == 0)
        return;

    ADC_DMA_configInfo dma_info;
    if (!ADC_DMA_getInfo(adc, &dma_info)) {
        return; /* Unsupported ADC */
    }
    /* Ensure DMA parameters are valid */
    DMA_Stream_TypeDef *dma_stream = dma_info.DMA_Stream;
    DMA_TypeDef *dma_peripheral = dma_info.DMAx;

    /* 1. Enable DMA Peripheral Clock */
    if (!(RCC->AHB1ENR & dma_info.RCC_AHB1ENR_DMAxEN)) {
        RCC->AHB1ENR |= dma_info.RCC_AHB1ENR_DMAxEN;
        /* Ensure the write is completed */
        (void) RCC->AHB1ENR;
    }

    /* 2. Configure ADC for DMA mode */
    /* Ensure ADC is disabled before changing DMA-related ADC settings */
    if (adc->CR2 & ADC_CR2_ADON) {
        adc->CR2 &= ~ADC_CR2_ADON;
        simple_delay(100); // Wait for ADOFF
    }

    /* Enable DMA mode for ADC */
    adc->CR2 |= ADC_CR2_DMA;
    /* Enable Continuous conversion mode */
    adc->CR2 |= ADC_CR2_CONT;

    /* DDS bit: DMA disable selection.
     * For STM32F42x/43x and newer, if DDS=1, DMA requests are issued as long as data are converted.
     * If DDS=0 (legacy or default for some F4s), DMA might stop after NDTR items even in circular mode.
     * For reliable circular DMA, DDS=1 is preferred if available.
     * Check your specific STM32F4 series Reference Manual for DDS bit availability and behavior.
     */
#if defined(ADC_CR2_DDS) // Check if DDS bit is defined for the target MCU
    if (dmaMode == ADC_DMA_MODE_CIRCULAR) {
        adc->CR2 |= ADC_CR2_DDS;
    } else {
        adc->CR2 &= ~ADC_CR2_DDS;
    }
#endif

    /* EOCS should be 0 for DMA to get request after each conversion */
    adc->CR2 &= ~ADC_CR2_EOCS;

    /* Disable ADC's own EOC interrupt if DMA is handling data transfer, to avoid redundant interrupts. */
    adc->CR1 &= ~ADC_CR1_EOCIE;

    /* 3. Configure DMA Stream */
    /* Disable DMA Stream before configuration */
    if (dma_stream->CR & DMA_SxCR_EN) {
        dma_stream->CR &= ~DMA_SxCR_EN;
        /* Wait for EN bit to be cleared */
        while (dma_stream->CR & DMA_SxCR_EN) {
            simple_delay(10);
        }
    }

    /* Clear all interrupt flags for the stream */
    if (dma_info.DMA_streamNumber <= 3) {
        /* Streams 0-3 use LIFCR (Low Interrupt Flag Clear Register) */
        dma_peripheral->LIFCR |= (0x3FUL << (dma_info.DMA_streamNumber * 6));
    } else {
        /* Streams 4-7 use HIFCR (High Interrupt Flag Clear Register) */
        dma_peripheral->HIFCR |= (0x3FUL
                << ((dma_info.DMA_streamNumber - 4) * 6));
    }

    /* Set peripheral port register address */
    dma_stream->PAR = (uint32_t) &(adc->DR);

    /* Set memory address */
    dma_stream->M0AR = (uint32_t) pData;

    /* Set number of data items to transfer */
    dma_stream->NDTR = length;

    /* Configure DMA stream: channel, direction, circular mode, increment mode, data size */
    /* Reset CR register */
    dma_stream->CR = 0;
    /* Select channel */
    dma_stream->CR |= dma_info.DMA_Channel;
    /* Direction: Peripheral-to-memory */
    dma_stream->CR &= ~DMA_SxCR_DIR;
    /* Memory increment mode enabled */
    dma_stream->CR |= DMA_SxCR_MINC;
    /* Peripheral increment mode disabled */
    dma_stream->CR &= ~DMA_SxCR_PINC;

    /* Peripheral data size: 16-bit (01) */
    dma_stream->CR |= DMA_SxCR_PSIZE_0;
    dma_stream->CR &= ~DMA_SxCR_PSIZE_1;
    /* Memory data size: 16-bit (01) */
    dma_stream->CR |= DMA_SxCR_MSIZE_0;
    dma_stream->CR &= ~DMA_SxCR_MSIZE_1;

    if (dmaMode == ADC_DMA_MODE_CIRCULAR) {
        /* Circular mode */
        dma_stream->CR |= DMA_SxCR_CIRC;
    } else {
        /* Normal mode */
        dma_stream->CR &= ~DMA_SxCR_CIRC;
    }

    /* Priority level: High (10)*/
    dma_stream->CR |= DMA_SxCR_PL_1;
    dma_stream->CR &= ~DMA_SxCR_PL_0;

    /* FIFO control register (optional, can be left at reset for direct mode) */
    dma_stream->FCR = 0; // Disable FIFO, Direct mode operation
}

/**
 * @brief Enables the DMA stream for ADC conversions.
 * @param adc Pointer to the ADC peripheral.
 * This function enables the DMA stream configured for the ADC.
 */
void ADC_DMA_Enable(ADC_TypeDef *adc) {
    ADC_DMA_configInfo dma_info;
    if (!ADC_DMA_getInfo(adc, &dma_info)) {
        return; /* Unsupported ADC */
    }
    DMA_Stream_TypeDef *dma_stream = dma_info.DMA_Stream;
    /* Check if stream is not already enabled */
    if (!(dma_stream->CR & DMA_SxCR_EN)) {
        /* Enable the DMA stream */
        dma_stream->CR |= DMA_SxCR_EN;
    }
}

/**
 * @brief Disables the DMA stream for ADC conversions.
 * @param adc Pointer to the ADC peripheral.
 * This function disables the DMA stream configured for the ADC.
 */
void ADC_DMA_Disable(ADC_TypeDef *adc) {
    ADC_DMA_configInfo dma_info;
    /* Get DMA configuration info for the specified ADC */
    if (!ADC_DMA_getInfo(adc, &dma_info)) {
        return; /* Unsupported ADC */
    }

    DMA_Stream_TypeDef *dma_stream = dma_info.DMA_Stream;
    /* Check if stream is currently enabled */
    if (dma_stream->CR & DMA_SxCR_EN) {
        dma_stream->CR &= ~DMA_SxCR_EN; /* Clear EN bit to disable stream */
        /* Wait for the EN bit to be cleared by hardware, indicating the stream is fully disabled. */
        while (dma_stream->CR & DMA_SxCR_EN) {
            simple_delay(10); /* Short delay while polling */
        }
    }
}

/**
 * @brief Gets the status of a specific DMA flag for the ADC.
 * @param adc Pointer to the ADC peripheral.
 * @param generic_flag The generic DMA flag to check (e.g., ADC_DMA_FLAG_TC).
 * @return 1 if the flag is set, 0 if not.
 */
uint8_t ADC_DMA_getFlagStatus(ADC_TypeDef *adc,
        ADC_DMA_genericFlag generic_flag) {
    ADC_DMA_configInfo dma_info;
    /* Get DMA configuration info for the specified ADC */
    if (!ADC_DMA_getInfo(adc, &dma_info)) {
        return 0; /* Unsupported ADC */
    }
    /* Get the specific flag bitmask based on the generic flag and stream number */
    uint32_t dma_flag_bitmask = ADC_DMA_getSpecificFlag(generic_flag,
            dma_info.DMA_streamNumber);
    /* Check if the flag bitmask is valid */
    if (dma_flag_bitmask == 0) {
        return 0; // Invalid flag
    }
    DMA_TypeDef *dma_peripheral = dma_info.DMAx;
    uint32_t status_register_value;

    /* Determine which status register (LISR or HISR) to read based on the stream number.
     Streams 0-3 use LISR (Low Interrupt Status Register).
     Streams 4-7 use HISR (High Interrupt Status Register). */
    if (dma_info.DMA_streamNumber <= 3) {
        /* Low Interrupt Status Register */
        status_register_value = dma_peripheral->LISR;
    } else {
        /* High Interrupt Status Register */
        status_register_value = dma_peripheral->HISR;
    }
    /* Check if the specified flag bit(s) are set in the status register value */
    return ((status_register_value & dma_flag_bitmask) != 0);
}

/**
 * @brief Clears a specific DMA flag for the ADC.
 * @param adc Pointer to the ADC peripheral.
 * @param generic_flag The generic DMA flag to clear (e.g., ADC_DMA_FLAG_TC).
 * This function clears the specified DMA flag by writing to the appropriate DMA flag clear register.
 * It uses the DMAx_LIFCR or DMAx_HIFCR register depending on the stream number.
 */
void ADC_DMA_clearFlag(ADC_TypeDef *adc, ADC_DMA_genericFlag generic_flag) {
    ADC_DMA_configInfo dma_info;
    /* Get DMA configuration info for the specified ADC */
    if (!ADC_DMA_getInfo(adc, &dma_info)) {
        return; /* Unsupported ADC */
    }
    /* Get the specific flag bitmask based on the generic flag and stream number */
    uint32_t dma_clear_flag_bitmask = ADC_DMA_getSpecificFlag(generic_flag,
            dma_info.DMA_streamNumber);
    /* Check if the flag bitmask is valid */
    if (dma_clear_flag_bitmask == 0) {
        return;
    }
    DMA_TypeDef *dma_peripheral = dma_info.DMAx;

    /* Determine which flag clear register (LIFCR or HIFCR) to write to.
     Streams 0-3 use LIFCR (Low Interrupt Flag Clear Register).
     Streams 4-7 use HIFCR (High Interrupt Flag Clear Register). */
    if (dma_info.DMA_streamNumber <= 3) {
        /* Low Interrupt Flag Clear Register */
        dma_peripheral->LIFCR |= dma_clear_flag_bitmask;
    } else {
        /* High Interrupt Flag Clear Register */
        dma_peripheral->HIFCR |= dma_clear_flag_bitmask;
    }
}
