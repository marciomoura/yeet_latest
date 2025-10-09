/**
 * @file task_io.h
 * @brief Structured I/O definitions for embedded control tasks
 *
 * This header defines the data structures used to pass inputs and outputs
 * between the HAL layer (C) and the application layer (C++).
 *
 * @note All structures are designed to be compatible with both C and C++ code.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Input structure for yeet_task_1 (5 kHz control task)
 *
 * Contains ADC measurement data from the current control loop.
 * - AD7606: 8 channels of high-speed 16-bit ADC via FSMC (channels 1-8, but only 1-9 used)
 * - STM32F407ZG-CN: Internal ADC channels IN10-IN13 (PC0-PC3) for 12-bit ADC (channels 10-13)
 * - FPGA: 48 digital inputs via FSMC (debounced)
 *
 * ADC Channel Mapping (1-indexed array, index 0 unused):
 * - adc_channels[1-9]:  AD7606 external ADC (16-bit, ±10V or ±5V range)
 *   [1] = Va   (Phase A Voltage)
 *   [2] = Vb   (Phase B Voltage)
 *   [3] = Vc   (Phase C Voltage)
 *   [4] = Vfd  (Field Voltage)
 *   [5] = Ia   (Phase A Current)
 *   [6] = Ib   (Phase B Current)
 *   [7] = Ic   (Phase C Current)
 *   [8] = Ifd  (Field Current)
 *   [9] = Reserved (not used, maps to AD7606 channel 8)
 *
 * - adc_channels[10-13]: STM32 internal ADC (12-bit, 0-4095 range)
 *   [10] = ADC1_IN10 (PC0)
 *   [11] = ADC1_IN11 (PC1)
 *   [12] = ADC1_IN12 (PC2)
 *   [13] = ADC1_IN13 (PC3)
 */
typedef struct {
    /* Unified ADC channels array (1-indexed, index 0 unused) */
    uint16_t adc_channels[14]; /**< ADC channels 1-13 (index 0 unused for 1-based indexing) */

    /* FPGA Digital Inputs (48 channels, debounced, 5 kHz sampling via FSMC) */
    struct {
        bool inputs[48]; /**< Debounced digital input states (DI0-DI47) */
        bool data_valid; /**< Flag indicating if FPGA input data is valid */
    } fpga_inputs;
} task1_inputs_t;

/**
 * @brief Output structure for yeet_task_1 (5 kHz control task)
 *
 * Contains thyristor gate pulse commands and FPGA digital outputs.
 * - STM32F407ZG-CN: GPIO pins PC6-PC9, PB0-PB1 for gate pulses
 * - FPGA: 48 digital outputs via FSMC
 *
 * Each boolean represents a gate pulse output:
 * - true: Gate pulse active (GPIO HIGH)
 * - false: Gate pulse inactive (GPIO LOW)
 *
 * @note Gate pulse outputs are applied atomically using the BSRR register to avoid glitches.
 * @note FPGA outputs are written as a block to FSMC registers.
 */
typedef struct {
    /* Gate pulse outputs (direct GPIO control) */
    bool pulse_1; /**< Gate pulse 1 (PC6) */
    bool pulse_2; /**< Gate pulse 2 (PC7) */
    bool pulse_3; /**< Gate pulse 3 (PC8) */
    bool pulse_4; /**< Gate pulse 4 (PC9) */
    bool pulse_5; /**< Gate pulse 5 (PB0) */
    bool pulse_6; /**< Gate pulse 6 (PB1) */

    /* FPGA Digital Outputs (48 channels, 5 kHz update via FSMC) */
    struct {
        bool outputs[48];    /**< Digital output states (DO0-DO47) */
        bool update_outputs; /**< Flag: true = write outputs to FPGA this cycle */
    } fpga_outputs;
} task1_outputs_t;

#ifdef __cplusplus
}
#endif
