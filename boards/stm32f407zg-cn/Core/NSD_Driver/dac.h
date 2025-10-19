/**
 ******************************************************************************
 * @file    dac.h
 * @brief   AD5724 4-channel DAC driver for STM32F407 via SPI
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * AD5724 Configuration:
 * - 4 channels: A, B, C, D (12-bit resolution)
 * - Output range: ±5V or ±10V (configurable per channel)
 * - Interface: SPI1 (PA4-NSS, PA5-SCK, PA6-MISO, PA7-MOSI)
 * - Control pins: LDAC(PC4), CLR(PC5)
 *
 ******************************************************************************
 */

#ifndef __DAC_H
#define __DAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of DAC channels */
#define AD5724_NUM_CHANNELS 4

/** @brief DAC resolution (12-bit) */
#define AD5724_RESOLUTION 12
/** @brief Maximum DAC code value */
#define AD5724_MAX_CODE 65535

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief AD5724 channel enumeration
 */
typedef enum {
    AD5724_CH_A = 0, /**< Channel A */
    AD5724_CH_B = 1, /**< Channel B */
    AD5724_CH_C = 2, /**< Channel C */
    AD5724_CH_D = 3  /**< Channel D */
} ad5724_channel_t;

/**
 * @brief AD5724 voltage range selection
 */
typedef enum {
    AD5724_RANGE_0_5V = 0,    /**< 0V to +5V */
    AD5724_RANGE_0_10V = 1,   /**< 0V to +10V */
    AD5724_RANGE_0_11V = 2,   /**< 0V to +10.8V */
    AD5724_RANGE_N5_5V = 3,   /**< -5V to +5V */
    AD5724_RANGE_N10_10V = 4, /**< -10V to +10V */
    AD5724_RANGE_N11_11V = 5  /**< -10.8V to +10.8V */
} ad5724_range_t;

/**
 * @brief AD5724 driver instance structure
 */
typedef struct {
    ad5724_range_t range[AD5724_NUM_CHANNELS]; /**< Voltage range per channel */
    float last_voltage[AD5724_NUM_CHANNELS];   /**< Last output voltage per channel */
    bool initialized;                          /**< Initialization flag */
    uint32_t write_count;                      /**< Total writes performed */
} ad5724_instance_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize AD5724 driver instance
 * @param dac Pointer to AD5724 instance structure
 * @retval true if initialization successful
 * @retval false if initialization failed
 */
bool AD5724_Init(ad5724_instance_t* dac);

/**
 * @brief Configure AD5724 voltage range for a channel
 * @param dac Pointer to AD5724 instance
 * @param channel Channel to configure
 * @param range Voltage range selection
 * @retval true if configuration successful
 * @retval false if configuration failed
 */
bool AD5724_SetRange(ad5724_instance_t* dac, ad5724_channel_t channel, ad5724_range_t range);

/**
 * @brief Configure AD5724 voltage range for all channels
 * @param dac Pointer to AD5724 instance
 * @param range Voltage range selection
 */
void AD5724_SetRangeAll(ad5724_instance_t* dac, ad5724_range_t range);

/**
 * @brief Reset AD5724 DAC (clear all outputs)
 * @param dac Pointer to AD5724 instance
 */
void AD5724_Reset(ad5724_instance_t* dac);

/* ============================================================================
 * Function Prototypes - Output Control
 * ============================================================================ */

/**
 * @brief Set voltage output on a specific channel
 * @param dac Pointer to AD5724 instance
 * @param channel Channel to set
 * @param voltage_v Voltage value in volts
 * @retval true if output set successfully
 * @retval false if output failed (out of range or invalid)
 */
bool AD5724_SetVoltage(ad5724_instance_t* dac, ad5724_channel_t channel, float voltage_v);

/**
 * @brief Set raw DAC code on a specific channel
 * @param dac Pointer to AD5724 instance
 * @param channel Channel to set
 * @param code Raw DAC code (0-65535)
 * @retval true if output set successfully
 * @retval false if output failed
 */
bool AD5724_SetCode(ad5724_instance_t* dac, ad5724_channel_t channel, uint16_t code);

/**
 * @brief Power up a specific channel
 * @param dac Pointer to AD5724 instance
 * @param channel Channel to power up
 */
void AD5724_PowerUpChannel(ad5724_instance_t* dac, ad5724_channel_t channel);

/**
 * @brief Power down a specific channel
 * @param dac Pointer to AD5724 instance
 * @param channel Channel to power down
 */
void AD5724_PowerDownChannel(ad5724_instance_t* dac, ad5724_channel_t channel);

/**
 * @brief Power up all channels
 * @param dac Pointer to AD5724 instance
 */
void AD5724_PowerUpAll(ad5724_instance_t* dac);

/* ============================================================================
 * Function Prototypes - Data Access
 * ============================================================================ */

/**
 * @brief Get last set voltage for a channel
 * @param dac Pointer to AD5724 instance
 * @param channel Channel to query
 * @retval Last voltage value in volts
 */
static inline float AD5724_GetLastVoltage(const ad5724_instance_t* dac, ad5724_channel_t channel)
{
    if (dac == NULL || (uint32_t)channel >= AD5724_NUM_CHANNELS) {
        return 0.0f;
    }
    return dac->last_voltage[channel];
}

/**
 * @brief Convert voltage to DAC code based on range
 * @param voltage_v Voltage value in volts
 * @param range Voltage range setting
 * @retval DAC code (0-65535)
 */
uint16_t AD5724_VoltageToCode(float voltage_v, ad5724_range_t range);

/**
 * @brief Check if voltage is within valid range
 * @param voltage_v Voltage value in volts
 * @param range Voltage range setting
 * @retval true if voltage is within range
 * @retval false if voltage is out of range
 */
bool AD5724_IsVoltageInRange(float voltage_v, ad5724_range_t range);

#ifdef __cplusplus
}
#endif

#endif /* __DAC_H */
