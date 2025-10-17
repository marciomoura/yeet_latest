/**
 ******************************************************************************
 * @file    adc.h
 * @brief   AD7606 8-channel ADC driver for STM32F407 via FSMC
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * AD7606 Configuration:
 * - 8 channels: AI1-AI8 (16-bit simultaneous sampling)
 * - Input range: ±10V (configurable to ±5V via RANGE pin)
 * - Interface: FSMC Bank2 (0x64000000)
 * - Control pins: BUSY(PD13), RST(PF10), RANGE(PF9), CONVSTA(PF6), CONVSTB(PF7)
 *
 ******************************************************************************
 */

#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================
 * Memory Map and Register Definitions
 * ============================================================================ */

/** @brief AD7606 base address (FSMC Bank2) */
#define AD7606_BASE_ADDR 0x64000000U
/** @brief Data register offset */
#define AD7606_DATA_OFFSET 0x800U
/** @brief Complete data register address */
#define AD7606_DATA_ADDR (AD7606_BASE_ADDR + AD7606_DATA_OFFSET)

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of ADC channels */
#define AD7606_NUM_CHANNELS 8

/** @brief Conversion time in microseconds */
#define AD7606_CONVERSION_TIME_US 5

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief AD7606 channel enumeration
 */
typedef enum {
    AD7606_CH0 = 0, /**< Channel 0 */
    AD7606_CH1,     /**< Channel 1 */
    AD7606_CH2,     /**< Channel 2 */
    AD7606_CH3,     /**< Channel 3 */
    AD7606_CH4,     /**< Channel 4 */
    AD7606_CH5,     /**< Channel 5 */
    AD7606_CH6,     /**< Channel 6 */
    AD7606_CH7      /**< Channel 7 */
} ad7606_channel_t;

/**
 * @brief AD7606 voltage range selection
 */
typedef enum {
    AD7606_RANGE_5V = 0, /**< ±5V input range */
    AD7606_RANGE_10V = 1 /**< ±10V input range */
} ad7606_range_t;

/**
 * @brief AD7606 driver instance structure
 */
typedef struct {
    ad7606_range_t range;  /**< Current voltage range */
    bool initialized;      /**< Initialization flag */
    uint32_t sample_count; /**< Total samples acquired */
} ad7606_instance_t;

/**
 * @brief Single ADC reading structure
 */
typedef struct {
    uint16_t raw[AD7606_NUM_CHANNELS]; /**< Raw 16-bit ADC values */
    bool valid;                        /**< Data validity flag */
} ad7606_reading_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize AD7606 driver instance
 * @param adc Pointer to AD7606 instance structure
 * @retval true if initialization successful
 * @retval false if initialization failed
 */
bool AD7606_Init(ad7606_instance_t* adc);

/**
 * @brief Configure AD7606 voltage range
 * @param adc Pointer to AD7606 instance
 * @param range Voltage range selection
 */
void AD7606_SetRange(ad7606_instance_t* adc, ad7606_range_t range);

/**
 * @brief Reset AD7606 ADC
 * @param adc Pointer to AD7606 instance
 */
void AD7606_Reset(ad7606_instance_t* adc);

/* ============================================================================
 * Function Prototypes - Data Acquisition
 * ============================================================================ */

/**
 * @brief Start AD7606 conversion
 * @param adc Pointer to AD7606 instance
 */
void AD7606_StartConversion(ad7606_instance_t* adc);

/**
 * @brief Check if conversion is complete
 * @param adc Pointer to AD7606 instance
 * @retval true if conversion complete (BUSY pin low)
 * @retval false if still busy
 */
bool AD7606_IsConversionReady(const ad7606_instance_t* adc);

/**
 * @brief Wait for conversion to complete with timeout
 * @param adc Pointer to AD7606 instance
 * @param timeout_us Timeout in microseconds
 * @retval true if conversion completed within timeout
 * @retval false if timeout occurred
 */
bool AD7606_WaitConversion(const ad7606_instance_t* adc, uint32_t timeout_us);

/**
 * @brief Read all 8 channels from AD7606
 * @param adc Pointer to AD7606 instance
 * @param reading Pointer to reading structure to store results
 * @retval true if read successful
 * @retval false if read failed
 */
bool AD7606_Read(ad7606_instance_t* adc, ad7606_reading_t* reading);

/**
 * @brief Start conversion and read previous results (pipeline mode)
 * @param adc Pointer to AD7606 instance
 * @param reading Pointer to reading structure to store results
 * @retval true if read successful
 * @retval false if read failed
 */
bool AD7606_ReadAndStart(ad7606_instance_t* adc, ad7606_reading_t* reading);

/* ============================================================================
 * Function Prototypes - Data Access
 * ============================================================================ */

/**
 * @brief Get channel value from reading
 * @param reading Pointer to reading structure
 * @param channel Channel number
 * @retval Channel value
 */
static inline uint16_t AD7606_GetChannelValue(const ad7606_reading_t* reading, ad7606_channel_t channel)
{
    if (reading == NULL) {
        return 0;
    }
    return reading->raw[channel];
}

/**
 * @brief Convert raw ADC value to voltage
 * @param raw_value Raw 16-bit ADC value
 * @param range Voltage range setting
 * @retval Voltage value in volts
 */
float AD7606_RawToVoltage(uint16_t raw_value, ad7606_range_t range);

/* ============================================================================
 * Function Prototypes - On-Chip ADC (STM32 internal ADC)
 * ============================================================================ */

/**
 * @brief On-chip ADC instance structure
 */
typedef struct {
    uint16_t num_channels; /**< Number of channels configured (1-4) */
    bool initialized;      /**< Initialization flag */
} onchip_adc_t;

/**
 * @brief Initialize on-chip ADC
 * @param adc Pointer to on-chip ADC instance
 * @param num_channels Number of channels to configure (1-4)
 * @retval true if initialization successful
 * @retval false if initialization failed
 */
bool OnChipADC_Init(onchip_adc_t* adc, uint16_t num_channels);

/**
 * @brief Get on-chip ADC results
 * @param adc Pointer to on-chip ADC instance
 * @param results Pointer to array to store results (must have space for num_channels)
 */
void OnChipADC_GetResults(onchip_adc_t* adc, uint16_t* results);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
