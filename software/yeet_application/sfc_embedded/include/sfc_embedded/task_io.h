/**
 * @file task_io.h
 * @brief Generic I/O interface for embedded control tasks
 *
 * This header defines generic data structures for passing inputs and outputs
 * between the board support layer (hardware drivers in main.c) and the
 * application layer (SFC control algorithms in C++).
 *
 * Design Principles:
 * - Hardware-agnostic: No dependencies on board-specific drivers
 * - Const-correct: Tasks receive const inputs (read-only) and writable outputs
 * - Organized: Nested structs group related interfaces (analog, digital, fiber-optic)
 * - Generic: Suitable for any control application, not SFC-specific
 *
 * @note All structures are C-compatible and can be used from both C and C++ code.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "yeet/network_ios.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of analog input channels (AD7606) */
#define TASK_IO_NUM_ANALOG_INPUTS 8

/** @brief Number of digital input channels */
#define TASK_IO_NUM_DIGITAL_INPUTS 48

/** @brief Number of digital output channels */
#define TASK_IO_NUM_DIGITAL_OUTPUTS 48

/** @brief Number of analog output channels (AD5724 DAC) */
#define TASK_IO_NUM_ANALOG_OUTPUTS 4

/** @brief Maximum number of float payload words in fiber-optic packets */
#define TASK_IO_FIBER_MAX_PAYLOAD 4

/** @brief Maximum network packet size (bytes) */
#define TASK_IO_NETWORK_MAX_PACKET_SIZE 1460  // TCP MSS typical size

/* ============================================================================
 * Real-Time Task Input Structures
 * ============================================================================ */

/**
 * @brief Generic task input structure (real-time tasks)
 *
 * Contains all input interfaces available to real-time control tasks:
 * - Analog inputs: ADC measurements from sensors
 * - Digital inputs: Debounced discrete signals (buttons, switches, status flags)
 * - Fiber-optic RX: Inter-board communication data (high-speed, deterministic)
 *
 * Signal Mapping (Application-Specific):
 * The board support layer (main.c) is responsible for mapping hardware-specific
 * signals to these generic arrays. Applications define their own semantic meanings.
 *
 * Example Analog Input Mapping:
 *   analog_inputs.channels[0] = Phase A voltage
 *   analog_inputs.channels[1] = Phase B voltage
 *   analog_inputs.channels[2] = Phase C voltage
 *   analog_inputs.channels[3] = DC link current
 *   analog_inputs.channels[4-7] = Application-defined
 *
 * Example Digital Input Mapping:
 *   digital_inputs.signals[0] = Emergency stop button
 *   digital_inputs.signals[1] = Enable command
 *   digital_inputs.signals[2] = Fault reset button
 *   digital_inputs.signals[3-15] = Application-defined
 *
 * @note Network I/O is handled separately via network_inputs_t (non-real-time)
 */
typedef struct {
    /**
     * @brief Analog input interface (ADC measurements)
     */
    struct {
        uint16_t channels[TASK_IO_NUM_ANALOG_INPUTS]; /**< ADC channel values (raw counts) */
        bool valid;                                   /**< true = ADC data is valid */
    } analog_inputs;

    /**
     * @brief Digital input interface (discrete signals)
     */
    struct {
        bool signals[TASK_IO_NUM_DIGITAL_INPUTS]; /**< Digital input states (true = active) */
        bool valid;                               /**< true = digital input data is valid */
    } digital_inputs;

    /**
     * @brief Fiber-optic receive interface (inter-board communication)
     */
    struct {
        uint32_t payload[TASK_IO_FIBER_MAX_PAYLOAD]; /**< Received uint32_t data from remote board */
        uint8_t payload_length;                      /**< Number of valid uint32_t words in payload (0-4) */
        bool data_ready;                             /**< true = new data received this cycle */
        bool link_ok;                                /**< true = fiber-optic link healthy */
    } fiber_optic_rx;

} task_inputs_t;

/* ============================================================================
 * Real-Time Task Output Structures
 * ============================================================================ */

/**
 * @brief Generic task output structure (real-time tasks)
 *
 * Contains all output interfaces available to real-time control tasks:
 * - Digital outputs: Status LEDs, relay controls, discrete commands
 * - Thyristor pulses: Gate firing signals for 3-phase thyristor bridge
 * - Fiber-optic TX: Inter-board communication data (high-speed, deterministic)
 *
 * Signal Mapping (Application-Specific):
 * The board support layer (main.c) reads these generic outputs and applies
 * them to hardware-specific drivers.
 *
 * Example Digital Output Mapping:
 *   digital_outputs.signals[0] = Fault indicator LED
 *   digital_outputs.signals[1] = Running indicator LED
 *   digital_outputs.signals[2] = Contactor control
 *   digital_outputs.signals[16] = Communication status LED
 *   digital_outputs.signals[17] = Heartbeat LED (auto-toggled)
 *
 * @note Network I/O is handled separately via network_outputs_t (non-real-time)
 */
typedef struct {
    /**
     * @brief Digital output interface (discrete signals)
     */
    struct {
        bool signals[TASK_IO_NUM_DIGITAL_OUTPUTS]; /**< Digital output states (true = active) */
    } digital_outputs;

    /**
     * @brief Analog output interface (DAC outputs)
     *
     * Controls four analog output channels via AD5724 DAC:
     * - Channel A, B, C, D (application-defined usage)
     * - Voltage range: Configurable (±5V, ±10V, etc.)
     * - Resolution: 16-bit
     *
     * Example DAC Output Mapping:
     *   analog_outputs.channels[0] = Reference voltage setpoint
     *   analog_outputs.channels[1] = Current command
     *   analog_outputs.channels[2] = Test signal
     *   analog_outputs.channels[3] = Diagnostic output
     *
     * @note DAC outputs are applied in 1ms task (1 kHz update rate)
     * @note Board support layer handles voltage range conversion
     */
    struct {
        float channels[TASK_IO_NUM_ANALOG_OUTPUTS]; /**< DAC output voltages (volts) */
        bool valid;                                 /**< true = DAC outputs should be applied */
    } analog_outputs;

    /**
     * @brief Thyristor gate pulse interface (3-phase bridge control)
     *
     * Controls six thyristor arms for 3-phase converter:
     * - Three positive arms (A+, B+, C+)
     * - Three negative arms (A-, B-, C-)
     *
     * @note Pulses are applied directly to GPIO pins by board support layer
     * @note Application is responsible for commutation logic and safety
     */
    struct {
        bool a_plus;  /**< Phase A positive arm gate pulse */
        bool c_minus; /**< Phase C negative arm gate pulse */
        bool b_plus;  /**< Phase B positive arm gate pulse */
        bool a_minus; /**< Phase A negative arm gate pulse */
        bool c_plus;  /**< Phase C positive arm gate pulse */
        bool b_minus; /**< Phase B negative arm gate pulse */
    } thyristor_pulses;

    /**
     * @brief Fiber-optic transmit interface (inter-board communication)
     */
    struct {
        uint32_t payload[TASK_IO_FIBER_MAX_PAYLOAD]; /**< uint32_t data to transmit to remote board */
        uint8_t payload_length;                      /**< Number of valid uint32_t words to send (0-4) */
        bool transmit_request;                       /**< true = transmit this cycle */
    } fiber_optic_tx;

} task_outputs_t;

using network_inputs_t = yeet::telemetry::network_inputs_t;
using network_outputs_t = yeet::telemetry::network_outputs_t;

#ifdef __cplusplus
}
#endif
