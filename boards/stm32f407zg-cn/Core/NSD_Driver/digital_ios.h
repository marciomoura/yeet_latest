/**
 ******************************************************************************
 * @file    digital_ios.h
 * @brief   Generic Digital I/O Driver for STM32F407
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Digital I/O Configuration:
 * - 16 generic digital inputs (application-defined mapping)
 * - 17 generic digital outputs (application-defined mapping)
 * - Hardware debounce filtering for reliable input reading
 * - Automatic heartbeat indicator on output 17
 *
 * This module provides a generic hardware abstraction for digital I/O signals.
 * It uses the FSMC GPIO driver for low-level hardware access. Applications
 * should map the generic signals (digital_input_1, digital_output_1, etc.) to
 * their specific use cases.
 *
 ******************************************************************************
 */

#ifndef __DIGITAL_IOS_H
#define __DIGITAL_IOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fsmc_gpio_driver.h"

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of digital input signals */
#define DIO_NUM_INPUTS 16

/** @brief Number of digital output signals */
#define DIO_NUM_OUTPUTS 17

/** @brief Number of digital input bytes (for communication) */
#define DIO_NUM_INPUT_BYTES 2

/** @brief Number of digital output bytes (for communication) */
#define DIO_NUM_OUTPUT_BYTES 8

/** @brief Running indicator toggle rate (cycles) */
#define DIO_RUNNING_TOGGLE_CYCLES 100

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Digital input signals structure
 * @note  Generic signal names - application maps these to specific functions
 */
typedef struct {
    bool digital_input_1;  /**< Digital input 1 */
    bool digital_input_2;  /**< Digital input 2 */
    bool digital_input_3;  /**< Digital input 3 */
    bool digital_input_4;  /**< Digital input 4 */
    bool digital_input_5;  /**< Digital input 5 */
    bool digital_input_6;  /**< Digital input 6 */
    bool digital_input_7;  /**< Digital input 7 */
    bool digital_input_8;  /**< Digital input 8 */
    bool digital_input_9;  /**< Digital input 9 */
    bool digital_input_10; /**< Digital input 10 */
    bool digital_input_11; /**< Digital input 11 */
    bool digital_input_12; /**< Digital input 12 */
    bool digital_input_13; /**< Digital input 13 */
    bool digital_input_14; /**< Digital input 14 */
    bool digital_input_15; /**< Digital input 15 */
    bool digital_input_16; /**< Digital input 16 */
} dio_input_flags_t;

/**
 * @brief Digital output signals structure
 * @note  Generic signal names - application maps these to specific functions
 */
typedef struct {
    bool digital_output_1;  /**< Digital output 1 */
    bool digital_output_2;  /**< Digital output 2 */
    bool digital_output_3;  /**< Digital output 3 */
    bool digital_output_4;  /**< Digital output 4 */
    bool digital_output_5;  /**< Digital output 5 */
    bool digital_output_6;  /**< Digital output 6 */
    bool digital_output_7;  /**< Digital output 7 */
    bool digital_output_8;  /**< Digital output 8 */
    bool digital_output_9;  /**< Digital output 9 */
    bool digital_output_10; /**< Digital output 10 */
    bool digital_output_11; /**< Digital output 11 */
    bool digital_output_12; /**< Digital output 12 */
    bool digital_output_13; /**< Digital output 13 */
    bool digital_output_14; /**< Digital output 14 */
    bool digital_output_15; /**< Digital output 15 */
    bool digital_output_16; /**< Digital output 16 */
    bool digital_output_17; /**< Digital output 17 (running heartbeat) */
} dio_output_flags_t;

/**
 * @brief Digital I/O driver instance structure
 */
typedef struct {
    fsmc_gpio_instance_t* gpio_driver;         /**< Pointer to hardware GPIO driver */
    dio_input_flags_t inputs;                  /**< Input status flags */
    dio_output_flags_t outputs;                /**< Output status flags */
    uint8_t input_data[DIO_NUM_INPUT_BYTES];   /**< Raw input data bytes */
    uint8_t output_data[DIO_NUM_OUTPUT_BYTES]; /**< Raw output data bytes */
    uint16_t running_count;                    /**< Running indicator counter */
    bool initialized;                          /**< Initialization flag */
    uint32_t update_count;                     /**< Total updates performed */
} dio_instance_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize digital I/O driver instance
 * @param dio Pointer to DIO instance structure
 * @param gpio_driver Pointer to initialized FSMC GPIO driver instance
 * @retval true if initialization successful
 * @retval false if initialization failed
 */
bool DIO_Init(dio_instance_t* dio, fsmc_gpio_instance_t* gpio_driver);

/**
 * @brief Reset digital I/O driver state
 * @param dio Pointer to DIO instance
 */
void DIO_Reset(dio_instance_t* dio);

/* ============================================================================
 * Function Prototypes - Input Processing
 * ============================================================================ */

/**
 * @brief Read and process all digital inputs
 * @param dio Pointer to DIO instance
 * @note Performs hardware read, debouncing, and flag update
 */
void DIO_ReadInputs(dio_instance_t* dio);

/**
 * @brief Get input flags structure
 * @param dio Pointer to DIO instance
 * @retval Pointer to input flags (read-only)
 */
const dio_input_flags_t* DIO_GetInputFlags(const dio_instance_t* dio);

/**
 * @brief Get specific input flag by name
 * @param dio Pointer to DIO instance
 * @param flag_name Name of the flag to query
 * @retval Flag value
 */
bool DIO_GetInputFlag(const dio_instance_t* dio, const char* flag_name);

/* ============================================================================
 * Function Prototypes - Output Processing
 * ============================================================================ */

/**
 * @brief Update and write all digital outputs
 * @param dio Pointer to DIO instance
 * @note Maps internal flags to hardware outputs
 */
void DIO_WriteOutputs(dio_instance_t* dio);

/**
 * @brief Set output flags structure
 * @param dio Pointer to DIO instance
 * @param outputs Pointer to output flags to set
 */
void DIO_SetOutputFlags(dio_instance_t* dio, const dio_output_flags_t* outputs);

/**
 * @brief Set specific output flag by name
 * @param dio Pointer to DIO instance
 * @param flag_name Name of the flag to set
 * @param value Flag value
 */
void DIO_SetOutputFlag(dio_instance_t* dio, const char* flag_name, bool value);

/* ============================================================================
 * Function Prototypes - Combined Operations
 * ============================================================================ */

/**
 * @brief Complete digital I/O update cycle
 * @param dio Pointer to DIO instance
 * @note Reads inputs, updates flags, writes outputs
 */
void DIO_Update(dio_instance_t* dio);

/* ============================================================================
 * Function Prototypes - Status and Diagnostics
 * ============================================================================ */

/**
 * @brief Get raw input data bytes
 * @param dio Pointer to DIO instance
 * @param buffer Buffer to store input bytes (must have space for DIO_NUM_INPUT_BYTES)
 */
void DIO_GetInputData(const dio_instance_t* dio, uint8_t* buffer);

/**
 * @brief Get raw output data bytes
 * @param dio Pointer to DIO instance
 * @param buffer Buffer to store output bytes (must have space for DIO_NUM_OUTPUT_BYTES)
 */
void DIO_GetOutputData(const dio_instance_t* dio, uint8_t* buffer);

#ifdef __cplusplus
}
#endif

#endif /* __DIGITAL_IOS_H */
