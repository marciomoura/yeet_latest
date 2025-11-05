/**
 * @file host_io_mocks.hpp
 * @brief Mock I/O functions for host platform simulation
 *
 * Provides placeholder implementations for hardware I/O (ADC, DIO, etc.)
 * when running on host platform. These can be enhanced later with realistic
 * waveform generators, state machines, or external input files.
 *
 * Current Implementation:
 * - Returns safe placeholder values (0.0, false, etc.)
 * - Allows control loop to execute without hardware
 *
 * Future Enhancements:
 * - Sinusoidal waveform generators for grid voltages
 * - Ramp generators for DC current profiles
 * - State machine scenario playback
 * - External data file playback
 * - Real-time user input control
 */

#pragma once

#ifdef HOST_BUILD

#include <cstdint>

namespace yeet::application::sfc::host {

/**
 * @brief Mock ADC measurement structure
 *
 * Placeholder for AD7606 8-channel ADC measurements.
 * Currently returns safe default values.
 */
struct mock_adc_measurements {
    float grid_voltage_a_pu;   ///< Phase A grid voltage [pu] (placeholder: 1.0)
    float grid_voltage_b_pu;   ///< Phase B grid voltage [pu] (placeholder: 1.0)
    float grid_voltage_c_pu;   ///< Phase C grid voltage [pu] (placeholder: 1.0)
    float dc_link_current;     ///< DC link current [pu] (placeholder: 0.0)
    float dc_link_voltage_pu;  ///< DC link voltage [pu] (placeholder: 1.0)
    float machine_current_pu;  ///< Machine current [pu] (placeholder: 0.0)
    float reserved_1;          ///< Reserved ADC channel
    float reserved_2;          ///< Reserved ADC channel
};

/**
 * @brief Mock digital input structure
 *
 * Placeholder for FSMC GPIO digital inputs.
 * Currently returns safe default values (all disabled/inactive).
 */
struct mock_dio_inputs {
    // Control commands
    bool enable_command;       ///< Enable command (placeholder: false)
    bool start_command;        ///< Start command (placeholder: false)
    bool stop_command;         ///< Stop command (placeholder: false)
    bool fault_reset_command;  ///< Fault reset command (placeholder: false)

    // Status inputs
    bool thyristor_fault;           ///< Thyristor fault status (placeholder: false)
    bool grid_contactor_closed;     ///< Grid contactor status (placeholder: false)
    bool machine_contactor_closed;  ///< Machine contactor status (placeholder: false)

    // Mode selections
    bool local_mode;  ///< Local/remote mode (placeholder: true = local)
    bool test_mode;   ///< Test mode enable (placeholder: false)

    // TODO: Add all actual DIO inputs from task_io.h when integrating
};

/**
 * @brief Mock digital output structure
 *
 * Placeholder for FSMC GPIO digital outputs.
 * Outputs are written but have no effect on host platform.
 */
struct mock_dio_outputs {
    // Status outputs
    bool running_status;  ///< Running status LED
    bool fault_status;    ///< Fault status LED
    bool ready_status;    ///< Ready status LED

    // Control outputs
    bool grid_contactor_command;     ///< Grid contactor control
    bool machine_contactor_command;  ///< Machine contactor control
    bool cooling_fan_enable;         ///< Cooling fan control

    // TODO: Add all actual DIO outputs from task_io.h when integrating
};

/**
 * @brief Read mock ADC measurements
 *
 * Returns placeholder ADC values. Can be enhanced with:
 * - Sinusoidal waveform generation
 * - Time-varying profiles
 * - External file playback
 *
 * @return Mock ADC measurements structure
 */
mock_adc_measurements read_mock_adc();

/**
 * @brief Read mock digital inputs
 *
 * Returns placeholder DIO input states. Can be enhanced with:
 * - Keyboard/console input control
 * - State machine scenario playback
 * - External command file
 *
 * @return Mock DIO inputs structure
 */
mock_dio_inputs read_mock_dio();

/**
 * @brief Write mock digital outputs
 *
 * No-op on host platform. Can be enhanced with:
 * - Console logging of output states
 * - External file logging
 * - Visual display in terminal
 *
 * @param outputs Digital output states to write
 */
void write_mock_dio(const mock_dio_outputs& outputs);

/**
 * @brief Initialize mock I/O subsystem
 *
 * Optional initialization for future enhancements
 * (e.g., load waveform files, start input thread, etc.)
 */
void initialize_mock_io();

/**
 * @brief Update mock I/O state
 *
 * Called periodically to update time-varying mock data.
 * Currently a no-op, but can be enhanced to advance waveform generators.
 *
 * @param timestamp_ms Current simulation time in milliseconds
 */
void update_mock_io(std::uint32_t timestamp_ms);

}  // namespace yeet::application::sfc::host

#endif  // HOST_BUILD
