/**
 ******************************************************************************
 * @file    fpga_io_driver.h
 * @brief   FPGA Digital I/O driver for STM32F407 via FSMC
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * FPGA I/O Configuration:
 * - 48 Digital Inputs (DI0-DI47) with 2-cycle debounce filter
 * - 48 Digital Outputs (DO0-DO47)
 * - 3 LED outputs (Run, Fault, Alarm)
 * - Interface: FSMC Bank4 (0x6C000000)
 * - Update rate: 1-5 kHz typical
 *
 ******************************************************************************
 */

#ifndef FPGA_IO_DRIVER_H
#define FPGA_IO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* ============================================================================
 * Memory Map and Register Definitions
 * ============================================================================ */

/** @brief FPGA base address (FSMC Bank4) */
#define FPGA_BASE_ADDR 0x6C000000U

/* Digital Input Registers (READ ONLY) */
#define DI_REG0_OFFSET (201 * 2) /**< DI0-DI15 register offset */
#define DI_REG1_OFFSET (202 * 2) /**< DI16-DI31 register offset */
#define DI_REG2_OFFSET (203 * 2) /**< DI32-DI47 register offset */

#define DI_REG0_ADDR (FPGA_BASE_ADDR + DI_REG0_OFFSET)
#define DI_REG1_ADDR (FPGA_BASE_ADDR + DI_REG1_OFFSET)
#define DI_REG2_ADDR (FPGA_BASE_ADDR + DI_REG2_OFFSET)

/* Digital Output Registers (WRITE ONLY) */
#define DO_REG0_OFFSET (3 * 2) /**< DO0-DO15 register offset */
#define DO_REG1_OFFSET (4 * 2) /**< DO16-DO31 register offset */
#define DO_REG2_OFFSET (5 * 2) /**< DO32-DO47 register offset */

#define DO_REG0_ADDR (FPGA_BASE_ADDR + DO_REG0_OFFSET)
#define DO_REG1_ADDR (FPGA_BASE_ADDR + DO_REG1_OFFSET)
#define DO_REG2_ADDR (FPGA_BASE_ADDR + DO_REG2_OFFSET)

/* Control and LED Registers (WRITE ONLY) */
#define CTRL_REG_OFFSET (6 * 2) /**< Control register offset */
#define LED_REG_OFFSET  (7 * 2) /**< LED register offset */

#define CTRL_REG_ADDR (FPGA_BASE_ADDR + CTRL_REG_OFFSET)
#define LED_REG_ADDR  (FPGA_BASE_ADDR + LED_REG_OFFSET)

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of digital input channels */
#define FPGA_NUM_DI 48

/** @brief Number of digital output channels */
#define FPGA_NUM_DO 48

/** @brief Debounce filter delay cycles (2 = ~400µs @ 5kHz) */
#define FPGA_DEBOUNCE_CYCLES 2

/** @brief Number of I/O groups (16 bits each) */
#define FPGA_IO_GROUPS 3

/** @brief Number of bits per I/O group */
#define FPGA_BITS_PER_GROUP 16

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief LED state structure
 */
typedef struct {
    bool run;   /**< Run LED (typically blinking) */
    bool fault; /**< Fault LED (on when fault active) */
    bool alarm; /**< Alarm LED (on when alarm active) */
} fpga_led_t;

/**
 * @brief Digital input state structure
 * @note Contains both raw and debounced input states
 */
typedef struct {
    bool raw[FPGA_NUM_DI];       /**< Raw input state (no filtering) */
    bool debounced[FPGA_NUM_DI]; /**< Debounced input state (2-cycle filter) */
    uint32_t update_count;       /**< Total number of updates performed */
} fpga_di_t;

/**
 * @brief Digital output state structure
 */
typedef struct {
    bool state[FPGA_NUM_DO]; /**< Output state for each channel */
    uint32_t update_count;   /**< Total number of updates performed */
} fpga_do_t;

/**
 * @brief Complete FPGA I/O state structure
 */
typedef struct {
    fpga_di_t inputs;  /**< Digital inputs (48 channels) */
    fpga_do_t outputs; /**< Digital outputs (48 channels) */
    fpga_led_t leds;   /**< LED indicators (3 channels) */
} fpga_io_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize FPGA I/O driver
 * @note Call once after MX_FSMC_Init()
 * @note Initializes all data structures and performs initial read
 * @retval None
 */
void FPGA_IO_Init(void);

/**
 * @brief Reset FPGA I/O state to defaults
 * @note Clears all outputs and resets debounce filters
 * @retval None
 */
void FPGA_IO_Reset(fpga_io_t* io);

/* ============================================================================
 * Function Prototypes - Digital Inputs
 * ============================================================================ */

/**
 * @brief Read all digital inputs from FPGA
 * @param io Pointer to FPGA I/O structure
 * @note Reads all 48 inputs (3 registers) and updates raw[] array
 * @note Does NOT update debounced[] - call FPGA_IO_UpdateDebounce() for that
 * @retval None
 */
void FPGA_IO_ReadInputs(fpga_io_t* io);

/**
 * @brief Update debounce filter for all inputs
 * @param io Pointer to FPGA I/O structure
 * @note Call after FPGA_IO_ReadInputs() to apply 2-cycle filter
 * @note Updates debounced[] array based on raw[] state
 * @retval None
 */
void FPGA_IO_UpdateDebounce(fpga_io_t* io);

/**
 * @brief Get single digital input state (debounced)
 * @param io Pointer to FPGA I/O structure
 * @param channel Channel number (0-47)
 * @retval true if input is HIGH (debounced)
 * @retval false if input is LOW (debounced)
 */
bool FPGA_IO_GetInput(const fpga_io_t* io, uint8_t channel);

/**
 * @brief Get single digital input state (raw, no debounce)
 * @param io Pointer to FPGA I/O structure
 * @param channel Channel number (0-47)
 * @retval true if input is HIGH (raw)
 * @retval false if input is LOW (raw)
 */
bool FPGA_IO_GetInputRaw(const fpga_io_t* io, uint8_t channel);

/* ============================================================================
 * Function Prototypes - Digital Outputs
 * ============================================================================ */

/**
 * @brief Write all digital outputs to FPGA
 * @param io Pointer to FPGA I/O structure
 * @note Writes all 48 outputs (3 registers) from state[] array
 * @retval None
 */
void FPGA_IO_WriteOutputs(fpga_io_t* io);

/**
 * @brief Set single digital output
 * @param io Pointer to FPGA I/O structure
 * @param channel Channel number (0-47)
 * @param state Output state (true=HIGH, false=LOW)
 * @note Updates state[] array but does NOT write to FPGA
 * @note Call FPGA_IO_WriteOutputs() to apply changes
 * @retval None
 */
void FPGA_IO_SetOutput(fpga_io_t* io, uint8_t channel, bool state);

/**
 * @brief Get single digital output state
 * @param io Pointer to FPGA I/O structure
 * @param channel Channel number (0-47)
 * @retval Current output state (true=HIGH, false=LOW)
 */
bool FPGA_IO_GetOutput(const fpga_io_t* io, uint8_t channel);

/**
 * @brief Toggle single digital output
 * @param io Pointer to FPGA I/O structure
 * @param channel Channel number (0-47)
 * @note Updates state[] array but does NOT write to FPGA
 * @note Call FPGA_IO_WriteOutputs() to apply changes
 * @retval None
 */
void FPGA_IO_ToggleOutput(fpga_io_t* io, uint8_t channel);

/**
 * @brief Set multiple outputs at once
 * @param io Pointer to FPGA I/O structure
 * @param start_channel Starting channel number (0-47)
 * @param count Number of channels to set (1-48)
 * @param states Array of states to set
 * @note Updates state[] array but does NOT write to FPGA
 * @note Call FPGA_IO_WriteOutputs() to apply changes
 * @retval None
 */
void FPGA_IO_SetOutputs(fpga_io_t* io, uint8_t start_channel, uint8_t count, const bool* states);

/* ============================================================================
 * Function Prototypes - LED Control
 * ============================================================================ */

/**
 * @brief Write LED states to FPGA
 * @param io Pointer to FPGA I/O structure
 * @note Writes all 3 LED indicators
 * @retval None
 */
void FPGA_IO_WriteLEDs(fpga_io_t* io);

/**
 * @brief Set run LED state
 * @param io Pointer to FPGA I/O structure
 * @param state LED state (true=ON, false=OFF)
 * @note Updates LED structure but does NOT write to FPGA
 * @note Call FPGA_IO_WriteLEDs() to apply changes
 * @retval None
 */
void FPGA_IO_SetRunLED(fpga_io_t* io, bool state);

/**
 * @brief Set fault LED state
 * @param io Pointer to FPGA I/O structure
 * @param state LED state (true=ON, false=OFF)
 * @retval None
 */
void FPGA_IO_SetFaultLED(fpga_io_t* io, bool state);

/**
 * @brief Set alarm LED state
 * @param io Pointer to FPGA I/O structure
 * @param state LED state (true=ON, false=OFF)
 * @retval None
 */
void FPGA_IO_SetAlarmLED(fpga_io_t* io, bool state);

/* ============================================================================
 * Function Prototypes - Combined Operations
 * ============================================================================ */

/**
 * @brief Update all I/O (read inputs, update outputs, write LEDs)
 * @param io Pointer to FPGA I/O structure
 * @note Typical usage in periodic task (1-5 kHz):
 *       1. Reads all inputs with debounce
 *       2. Writes all outputs
 *       3. Writes LEDs
 *       4. Clears control register
 * @retval None
 */
void FPGA_IO_Update(fpga_io_t* io);

/* ============================================================================
 * Inline Helper Functions
 * ============================================================================ */

/**
 * @brief Check if channel number is valid
 * @param channel Channel number to check
 * @retval true if channel is valid (0-47)
 */
static inline bool FPGA_IO_IsValidChannel(uint8_t channel) { return (channel < FPGA_NUM_DI); }

/**
 * @brief Toggle run LED (for blinking effect)
 * @param io Pointer to FPGA I/O structure
 */
static inline void FPGA_IO_ToggleRunLED(fpga_io_t* io) { io->leds.run = !io->leds.run; }

#ifdef __cplusplus
}
#endif

#endif /* FPGA_IO_DRIVER_H */
