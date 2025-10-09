/**
 ******************************************************************************
 * @file    ad7606_driver.h
 * @brief   AD7606 8-channel ADC driver for STM32F407 via FSMC
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * AD7606 Configuration:
 * - 8 channels: AI1-AI8 (16-bit simultaneous sampling)
 * - Input range: ±10V (configurable to ±5V via RANGE pin)
 * - Interface: FSMC Bank2 (0x64000000)
 * - Control pins: PF6 (CONVSTA), PF7 (CONVSTB), PF9 (RANGE), PF10 (RESET), PD13 (BUSY)
 *
 ******************************************************************************
 */

#ifndef AD7606_DRIVER_H
#define AD7606_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"


/* ============================================================================
 * Memory Map and Register Definitions
 * ============================================================================ */

/** @brief AD7606 base address (FSMC Bank2) */
#define AD7606_BASE_ADDR 0x64000000U

/** @brief Data register offset (0x800 = 2048 bytes) */
#define AD7606_DATA_OFFSET 0x800U

/** @brief Complete data register address */
#define AD7606_DATA_ADDR (AD7606_BASE_ADDR + AD7606_DATA_OFFSET)

/* ============================================================================
 * GPIO Pin Definitions
 * ============================================================================ */

/** @brief CONVSTA/B conversion trigger pins (PF6, PF7) */
#define AD7606_CONVST_PORT GPIOF
#define AD7606_CONVSTA_PIN GPIO_PIN_6
#define AD7606_CONVSTB_PIN GPIO_PIN_7
#define AD7606_CONVST_PINS (AD7606_CONVSTA_PIN | AD7606_CONVSTB_PIN)

/** @brief RANGE pin (PF9) - High=±10V, Low=±5V */
#define AD7606_RANGE_PORT GPIOF
#define AD7606_RANGE_PIN  GPIO_PIN_9

/** @brief RESET pin (PF10) */
#define AD7606_RESET_PORT GPIOF
#define AD7606_RESET_PIN  GPIO_PIN_10

/** @brief BUSY pin (PD13) - input, conversion status */
#define AD7606_BUSY_PORT GPIOD
#define AD7606_BUSY_PIN  GPIO_PIN_13

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of ADC channels */
#define AD7606_NUM_CHANNELS 8

/** @brief Conversion time in microseconds (~3.6 µs typical) */
#define AD7606_CONVERSION_TIME_US 5

/** @brief Voltage range for ±10V mode */
#define AD7606_VOLTAGE_RANGE_10V 10.0f

/** @brief Voltage range for ±5V mode */
#define AD7606_VOLTAGE_RANGE_5V 5.0f

/** @brief ADC resolution (16-bit signed) */
#define AD7606_MAX_COUNT 32768

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief AD7606 channel enumeration
 * @note Matches physical wiring per documentation
 */
typedef enum {
    AD7606_CH0_VA = 0, /**< Channel 0: Phase A Voltage */
    AD7606_CH1_VB,     /**< Channel 1: Phase B Voltage */
    AD7606_CH2_VC,     /**< Channel 2: Phase C Voltage */
    AD7606_CH3_VFD,    /**< Channel 3: Field Voltage */
    AD7606_CH4_IA,     /**< Channel 4: Phase A Current */
    AD7606_CH5_IB,     /**< Channel 5: Phase B Current */
    AD7606_CH6_IC,     /**< Channel 6: Phase C Current */
    AD7606_CH7_IFD     /**< Channel 7: Field Current */
} ad7606_channel_t;

/**
 * @brief AD7606 voltage range selection
 */
typedef enum {
    AD7606_RANGE_5V = 0, /**< ±5V input range */
    AD7606_RANGE_10V = 1 /**< ±10V input range */
} ad7606_range_t;

/**
 * @brief AD7606 data structure
 * @note Contains both raw ADC counts and converted voltage values
 */
typedef struct {
    uint16_t raw[AD7606_NUM_CHANNELS];  /**< Raw 16-bit ADC values */
    float voltage[AD7606_NUM_CHANNELS]; /**< Converted voltages in volts */
    ad7606_range_t range;               /**< Current voltage range setting */
    bool conversion_ready;              /**< Flag: data is valid */
    uint32_t conversion_count;          /**< Total conversions performed */
} ad7606_data_t;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize AD7606 ADC
 * @note Call once after MX_FSMC_Init() and MX_GPIO_Init()
 * @note STM32CubeMX already configured FSMC and GPIO pins
 * @retval None
 */
void AD7606_Init(void);

/**
 * @brief Configure AD7606 voltage range
 * @param range Voltage range selection (±5V or ±10V)
 * @retval None
 */
void AD7606_SetRange(ad7606_range_t range);

/**
 * @brief Reset AD7606 ADC
 * @note Performs hardware reset sequence
 * @retval None
 */
void AD7606_Reset(void);

/**
 * @brief Start AD7606 conversion
 * @note Triggers simultaneous sampling on all 8 channels
 * @note Conversion takes ~3.6 µs
 * @retval None
 */
void AD7606_StartConversion(void);

/**
 * @brief Check if AD7606 conversion is complete
 * @retval true if conversion complete (BUSY = LOW)
 * @retval false if conversion in progress (BUSY = HIGH)
 */
bool AD7606_IsConversionReady(void);

/**
 * @brief Read all 8 channels from AD7606
 * @param data Pointer to ad7606_data_t structure to store results
 * @note Reads raw values and converts to voltage
 * @note Must be called after conversion is complete
 * @retval None
 */
void AD7606_Read(ad7606_data_t* data);

/**
 * @brief Start conversion and read previous results (pipeline mode)
 * @param data Pointer to ad7606_data_t structure to store results
 * @note More efficient - minimizes dead time between conversions
 * @note First call will return zeros (no previous conversion)
 * @retval None
 */
void AD7606_ReadAndStart(ad7606_data_t* data);

/**
 * @brief Convert raw ADC value to voltage
 * @param raw_value Raw 16-bit ADC count (signed)
 * @param range Voltage range (±5V or ±10V)
 * @retval Voltage in volts (±5.0V or ±10.0V)
 */
float AD7606_RawToVoltage(uint16_t raw_value, ad7606_range_t range);

/**
 * @brief Wait for conversion to complete (blocking)
 * @param timeout_us Timeout in microseconds (0 = no timeout)
 * @retval true if conversion completed
 * @retval false if timeout occurred
 */
bool AD7606_WaitConversion(uint32_t timeout_us);

/* ============================================================================
 * Inline Helper Functions
 * ============================================================================ */

/**
 * @brief Get voltage for specific channel
 * @param data Pointer to AD7606 data structure
 * @param channel Channel number (0-7)
 * @retval Voltage in volts
 */
static inline float AD7606_GetChannelVoltage(const ad7606_data_t* data, ad7606_channel_t channel)
{
    return data->voltage[channel];
}

/**
 * @brief Get raw count for specific channel
 * @param data Pointer to AD7606 data structure
 * @param channel Channel number (0-7)
 * @retval Raw 16-bit ADC count
 */
static inline uint16_t AD7606_GetChannelRaw(const ad7606_data_t* data, ad7606_channel_t channel)
{
    return data->raw[channel];
}

#ifdef __cplusplus
}
#endif

#endif /* AD7606_DRIVER_H */
