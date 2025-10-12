// Copyright 2025 Techperts
// Runner for SFC Slow IO component

#pragma once

#include "sfc_integration/sfc_slow_io_datamodel.hpp"
#include "sfc_integration/sfc_slow_io_interface.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Runner for slow I/O operations at 100 Hz (10 ms sampling)
 *
 * This component bridges the hardware abstraction layer (raw bitfield structs)
 * to the application layer (signal_input/output system). It performs:
 *
 * 1. **Input Processing**: Reads raw hardware states and publishes to signal outputs
 *    - Digital inputs: TCS commands, protection signals, breaker status
 *    - Analog inputs: Excitation current/voltage feedback
 *
 * 2. **Output Processing**: Reads signal inputs and writes to raw hardware commands
 *    - Digital outputs: Fan control, breaker commands, TCS feedback, excitation control
 *    - Analog outputs: Excitation voltage/current references
 *
 * 3. **Composite Signal Generation**: Creates derived signals from raw inputs
 *    - Auxiliaries health: Aggregates multiple protection signals
 *    - Breaker ready: Checks breaker status and safety interlocks
 *    - Shutdown request: OR's all emergency trip signals
 *    - Excitation field ready: Threshold detection on current + voltage
 *
 * Memory footprint: 28 bytes for raw I/O state (8+4+8+8)
 * Update rate: 10 ms (100 Hz) - suitable for supervisory control
 */
class sfc_slow_io_runner : public yeet::base_application<sfc_slow_io_datamodel::inputs,
                               sfc_slow_io_datamodel::outputs,
                               sfc_slow_io_datamodel::parameters> {
public:
    using datamodel = sfc_slow_io_datamodel;

    /// Sampling time: 10 ms (100 Hz) for slow supervisory I/O
    static constexpr duration_t sampling_time = ms(10);

    /**
     * @brief Construct slow IO runner
     *
     * Initializes raw I/O structs and wrapper objects. All outputs are set to safe
     * state (fans off, breakers open, excitation off, zero references).
     */
    sfc_slow_io_runner();

    /**
     * @brief Initialize slow IO operations
     *
     * Called once before first run(). Sets all outputs to safe initial state.
     */
    void initialize() override;

    /**
     * @brief Execute one slow IO cycle (10 ms period)
     *
     * Processing order:
     * 1. Read raw digital/analog inputs from hardware interface
     * 2. Process inputs and publish to signal outputs (for other components to read)
     * 3. Read signal inputs (commands from other components)
     * 4. Process outputs and write to raw hardware interface
     */
    void run() override;

    /**
     * @brief Stop slow IO operations
     *
     * Sets all outputs to safe state (fans off, excitation off, breakers safe).
     */
    void stop() override;

    /**
     * @brief Get pointer to digital input raw struct for hardware layer
     *
     * The hardware layer (e.g., main.c in STM32 firmware) should call this
     * to get the struct pointer, then fill it with data read from FPGA GPIO
     * and external ADC before each run() cycle.
     *
     * @return Pointer to raw digital input bitfield struct (8 bytes)
     */
    ::yeet::sfc::sfc_digital_input_raw* get_digital_input_raw_ptr() noexcept { return &_digital_input_raw; }

    /**
     * @brief Get pointer to digital output raw struct for hardware layer
     *
     * The hardware layer should call this after each run() cycle to get
     * the updated output commands, then write them to FPGA GPIO.
     *
     * @return Pointer to raw digital output bitfield struct (4 bytes)
     */
    const ::yeet::sfc::sfc_digital_output_raw* get_digital_output_raw_ptr() const noexcept { return &_digital_output_raw; }

    /**
     * @brief Get pointer to analog input raw struct for hardware layer
     *
     * The hardware layer should call this to get the struct pointer, then
     * fill it with data read from external ADC (e.g., AD7606).
     *
     * @return Pointer to raw analog input struct (8 bytes)
     */
    ::yeet::sfc::sfc_analog_input_raw* get_analog_input_raw_ptr() noexcept { return &_analog_input_raw; }

    /**
     * @brief Get pointer to analog output raw struct for hardware layer
     *
     * The hardware layer should call this after each run() cycle to get
     * the updated analog references (DAC outputs or network transmission).
     *
     * @return Pointer to raw analog output struct (8 bytes)
     */
    const ::yeet::sfc::sfc_analog_output_raw* get_analog_output_raw_ptr() const noexcept { return &_analog_output_raw; }

private:
    // ===== Raw I/O Structs (Hardware Interface Layer) =====
    /// Raw digital inputs from hardware (8 bytes)
    ::yeet::sfc::sfc_digital_input_raw _digital_input_raw{};

    /// Raw digital outputs to hardware (4 bytes)
    ::yeet::sfc::sfc_digital_output_raw _digital_output_raw{};

    /// Raw analog inputs from hardware (8 bytes)
    ::yeet::sfc::sfc_analog_input_raw _analog_input_raw{};

    /// Raw analog outputs to hardware (8 bytes)
    ::yeet::sfc::sfc_analog_output_raw _analog_output_raw{};

    // ===== Semantic Wrapper Objects (Application Layer) =====
    /// Semantic wrapper for digital inputs (no storage, references _digital_input_raw)
    ::yeet::sfc::sfc_digital_inputs _digital_inputs;

    /// Semantic wrapper for digital outputs (no storage, references _digital_output_raw)
    ::yeet::sfc::sfc_digital_outputs _digital_outputs;

    /// Semantic wrapper for analog inputs (no storage, references _analog_input_raw)
    ::yeet::sfc::sfc_analog_inputs _analog_inputs;

    /// Semantic wrapper for analog outputs (no storage, references _analog_output_raw)
    ::yeet::sfc::sfc_analog_outputs _analog_outputs;

    // ===== Private Helper Methods =====

    /**
     * @brief Process input signals and publish to outputs
     *
     * Reads from _digital_inputs and _analog_inputs, publishes to _outputs
     * signal_output members for other components to read.
     */
    void process_inputs();

    /**
     * @brief Process output commands and write to hardware
     *
     * Reads from _inputs signal_input members (commands from other components),
     * writes to _digital_outputs and _analog_outputs for hardware layer.
     */
    void process_outputs();

    /**
     * @brief Compute auxiliaries health status
     *
     * Aggregates multiple protection and interlock signals into a single
     * "auxiliaries are healthy" status for state machine.
     *
     * @return true if all auxiliary systems are operational
     */
    bool compute_auxiliaries_healthy() const noexcept;

    /**
     * @brief Compute shutdown request status
     *
     * OR's all emergency trip signals (E-stop, hardware trip, external fault,
     * severe protection trips).
     *
     * @return true if any shutdown condition is present
     */
    bool compute_shutdown_requested() const noexcept;

    /**
     * @brief Compute excitation field established status
     *
     * Checks if both field current AND field voltage exceed configured
     * thresholds (indicating field winding flux is established).
     *
     * @return true if excitation field is ready for operation
     */
    bool compute_excitation_field_ready() const noexcept;
};

}  // namespace yeet::application::sfc::integration

