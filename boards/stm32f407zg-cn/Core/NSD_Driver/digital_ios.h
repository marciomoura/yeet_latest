/**
 ******************************************************************************
 * @file    digital_ios.h
 * @brief   Digital I/O status monitoring driver for STM32F407
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Digital I/O Configuration:
 * - Input monitoring: Fuses, temperature, breaker, fan status
 * - Output control: SCR errors, sync faults, optical communication status
 * - Debounce filtering for reliable input reading
 * - Status flag mapping for external monitoring equipment
 *
 * This module provides application-specific interpretation of hardware GPIO
 * signals. It uses the FSMC GPIO driver for low-level hardware access.
 *
 ******************************************************************************
 */

#ifndef __DIGITAL_IOS_H
#define __DIGITAL_IOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fsmc_gpio_driver.h"

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of digital input bytes */
#define DIO_NUM_INPUT_BYTES 2

/** @brief Number of digital output bytes */
#define DIO_NUM_OUTPUT_BYTES 8

/** @brief Number of bits per byte */
#define DIO_BITS_PER_BYTE 8

/** @brief Number of fuse arms monitored */
#define DIO_NUM_FUSE_ARMS 6

/** @brief Number of optical communication channels */
#define DIO_NUM_OPTICAL_CHANNELS 8

/** @brief Running indicator toggle rate (cycles) */
#define DIO_RUNNING_TOGGLE_CYCLES 100

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Digital input flags structure
 */
typedef struct {
    bool fuse_flag;             /**< Combined fuse status (any arm) */
    bool arm_fuse_flag[6];      /**< Individual arm fuse flags */
    bool over_temperature_flag; /**< Over temperature fault */
    bool fan_flag;              /**< Cooling fan status */
    bool ref_up_flag;           /**< Reference up command */
    bool ref_down_flag;         /**< Reference down command */
    bool stop_command;          /**< Stop command */
    bool pulse_enabled_flag;    /**< Pulse enable state */
    bool breaker_flag;          /**< Circuit breaker status */
    bool pulse_power_flag;      /**< Pulse power supply fault */
} dio_input_flags_t;

/**
 * @brief Digital output flags structure
 */
typedef struct {
    bool scr_error_flag; /**< SCR (thyristor) error */
    bool fuse_flag;      /**< Fuse fault */
    bool fan_on_flag;    /**< Fan on status */
    bool sec_a_flag;     /**< Phase A secondary fault */
    bool sec_b_flag;     /**< Phase B secondary fault */
    bool sec_c_flag;     /**< Phase C secondary fault */
    bool syn_a_err_flag; /**< Phase A sync error */
    bool syn_b_err_flag; /**< Phase B sync error */
    bool syn_c_err_flag; /**< Phase C sync error */
    bool rom_flag;       /**< ROM/Flash memory status */
    bool scr_exit_flag;  /**< SCR exit flag */
    bool low_f_flag;     /**< Low frequency flag */
    bool over_ex_flag;   /**< Over excitation flag */
    bool pt_break_flag;  /**< PT (potential transformer) disconnection */
    bool standby_flag;   /**< Standby mode flag */
    bool online_flag;    /**< Online communication active */
    bool running_flag;   /**< System running heartbeat */
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
    int16_t do_lock_delay;                     /**< Output lock delay counter */
    bool do_lock_flag;                         /**< Output lock flag */
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

/**
 * @brief Set output lock (prevents output updates)
 * @param dio Pointer to DIO instance
 * @param delay_cycles Number of cycles to lock outputs
 */
void DIO_SetOutputLock(dio_instance_t* dio, int16_t delay_cycles);

/**
 * @brief Check if outputs are locked
 * @param dio Pointer to DIO instance
 * @retval true if outputs are locked
 * @retval false if outputs are not locked
 */
static inline bool DIO_IsOutputLocked(const dio_instance_t* dio)
{
    if (dio == NULL) {
        return false;
    }
    return dio->do_lock_flag;
}

/* ============================================================================
 * Legacy Function Prototypes (backward compatibility)
 * ============================================================================ */

/**
 * @brief Legacy DIDO update function
 * @note Preserved for backward compatibility with existing code
 */
void DIDO(void);

#ifdef __cplusplus
}
#endif

#endif /* __DIGITAL_IOS_H */
