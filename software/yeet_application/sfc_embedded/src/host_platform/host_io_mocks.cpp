/**
 * @file host_io_mocks.cpp
 * @brief Implementation of mock I/O functions for host platform
 *
 * Provides safe placeholder implementations that allow control algorithms
 * to execute without hardware. Can be enhanced to generate realistic
 * waveforms and scenarios.
 */

#ifdef HOST_BUILD

#include "sfc_embedded/host_platform/host_io_mocks.hpp"

#include <cmath>

namespace yeet::application::sfc::host {

// Static state for time-varying mock data (future enhancement)
namespace {
bool mock_io_initialized = false;
std::uint32_t last_timestamp_ms = 0;

// Future: Waveform generator state
// float phase_angle = 0.0f;
// float frequency_hz = 50.0f;
}  // namespace

mock_adc_measurements read_mock_adc()
{
    // Return safe placeholder values
    // Grid voltages at nominal (1.0 pu) for stable operation
    // DC link current at zero (no load)
    // DC link voltage at nominal (1.0 pu)

    mock_adc_measurements measurements;
    measurements.grid_voltage_a_pu = 1.0f;   // Nominal grid voltage
    measurements.grid_voltage_b_pu = 1.0f;   // Nominal grid voltage
    measurements.grid_voltage_c_pu = 1.0f;   // Nominal grid voltage
    measurements.dc_link_current = 0.0f;     // No load current
    measurements.dc_link_voltage_pu = 1.0f;  // Nominal DC voltage
    measurements.machine_current_pu = 0.0f;  // No machine current
    measurements.reserved_1 = 0.0f;
    measurements.reserved_2 = 0.0f;

    // Future enhancement: Generate sinusoidal waveforms
    // float time_s = last_timestamp_ms * 0.001f;
    // float omega = 2.0f * M_PI * frequency_hz;
    // measurements.grid_voltage_a_pu = std::sin(omega * time_s);
    // measurements.grid_voltage_b_pu = std::sin(omega * time_s - 2.0f * M_PI / 3.0f);
    // measurements.grid_voltage_c_pu = std::sin(omega * time_s + 2.0f * M_PI / 3.0f);

    return measurements;
}

mock_dio_inputs read_mock_dio()
{
    // Return safe placeholder values
    // All commands disabled, all faults inactive
    // Local mode enabled for safe operation

    mock_dio_inputs inputs;

    // Control commands - all disabled by default
    inputs.enable_command = false;
    inputs.start_command = false;
    inputs.stop_command = false;
    inputs.fault_reset_command = false;
    inputs.confirm_alarms = false;

    // Status inputs - safe states
    inputs.thyristor_fault = false;        // No fault
    inputs.grid_contactor_closed = false;  // Contactors open (safe)
    inputs.machine_contactor_closed = false;

    // Mode selections
    inputs.local_mode = true;  // Local mode (safe)
    inputs.test_mode = false;  // Normal mode

    // Future enhancement: Read from keyboard, file, or network
    // Example: Check for 'e' key press to set enable_command = true
    // Example: Read scenario file to set input sequence

    return inputs;
}

void write_mock_dio(const mock_dio_outputs& outputs)
{
    // No-op on host platform
    // Future enhancement: Log to console or file

    // Example logging (commented out for now):
    // if (outputs.running_status) {
    //     printf("[DIO] Running status ON\n");
    // }
    // if (outputs.fault_status) {
    //     printf("[DIO] FAULT status ON\n");
    // }

    (void)outputs;  // Suppress unused parameter warning
}

void initialize_mock_io()
{
    if (mock_io_initialized) {
        return;  // Already initialized
    }

    // Future enhancement: Initialize waveform generators, load files, etc.
    // Example: Load scenario file
    // Example: Start keyboard input thread
    // Example: Initialize random number generator

    mock_io_initialized = true;
}

void update_mock_io(std::uint32_t timestamp_ms)
{
    last_timestamp_ms = timestamp_ms;

    // Future enhancement: Update time-varying mock data
    // Example: Advance waveform generator phase
    // Example: Check scenario timeline for next event
    // Example: Update ramp generators

    // Calculate time delta for waveform generators
    // std::uint32_t delta_ms = (last_timestamp_ms == 0) ? 0 : (timestamp_ms - last_timestamp_ms);
    // float delta_s = delta_ms * 0.001f;
    // phase_angle += 2.0f * M_PI * frequency_hz * delta_s;
    // if (phase_angle > 2.0f * M_PI) phase_angle -= 2.0f * M_PI;
}

}  // namespace yeet::application::sfc::host

#endif  // HOST_BUILD
