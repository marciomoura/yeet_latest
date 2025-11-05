#pragma once

#include <array>

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_external_ios_datamodel.hpp"
#include "sfc_integration/sfc_grid_side_integrator_supervisor_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"
#include "sfc_integration/sfc_per_unit_system_datamodel.hpp"
#include "sfc_integration/sfc_state_machine_runner.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Grid-side control system for Static Frequency Converter
 *
 * This class encapsulates all grid-side control components including:
 * - State machine for system control
 * - Current controller for DC link regulation
 * - Grid-side observer for voltage/angle estimation
 * - Grid-side modulator and pulse generator
 * - Measurement processing for grid and DC link
 * - Operation mode monitoring
 *
 * Signal Interface:
 * All external signals are accessed through the `get_external_ios()` datamodel interface,
 * which provides abstraction for digital inputs, analog inputs, and fiber-optic signals.
 */
class sfc_grid_side_control {
public:
    /**
     * @brief Construct grid-side control with per-unit system
     * @param per_unit_system Shared per-unit system for scaling
     */
    explicit sfc_grid_side_control()
        : state_machine(),
          grid_integrator_supervisor(),
          current_controller(per_unit_system),
          grid_side_observer(),
          grid_side_modulator(),
          grid_side_pulse_generator(),
          grid_measurements(per_unit_system),
          dc_link_measurements(per_unit_system),
          operation_mode_monitor()
    {
        connect_internal_signals();
    }

    /**
     * @brief Initialize all grid-side components
     */
    void initialize();

    /**
     * @brief Reset all grid-side components to initial state
     */
    void reset();

    /**
     * @brief Register all component datamodels with the model registry
     *
     * This registers all parameters from grid-side components (current controller,
     * observer, modulator, pulse generator, measurements, state machine, operation mode monitor)
     * with the provided registry, making them accessible via parameter configuration protocol.
     *
     * @param registry The model registry to populate with component parameters
     */
    void register_datamodel(yeet::model_registry& registry);

    /**
     * @brief Update components running at 10ms sampling time (100 Hz)
     */
    void update_10ms();

    /**
     * @brief Update components running at 1ms sampling time (1 kHz)
     *
     */
    void update_1ms();

    /**
     * @brief Update components running at 200µs sampling time (5 kHz)
     *
     */
    void update_200us();

    /**
     * @brief Encode grid-side signals into fiber-optic protocol for transmission to machine-side
     *
     * Collects operating mode, DC link current, and status flags, then encodes them
     * into a 2-word (8 byte) fiber-optic message using the standardized protocol.
     * The encoded payload is stored internally in _fiber_optic_tx_payload.
     *
     * @return Number of uint32_t words written (always 2)
     */
    std::uint8_t encode_fiber_optic_tx_to_machine();

    /**
     * @brief Get the encoded fiber-optic TX payload for transmission to machine-side
     *
     * @return Pointer to internal payload buffer (always 2 words for grid→machine)
     */
    const std::uint32_t* get_fiber_optic_tx_payload() const { return _fiber_optic_tx_payload.data(); }

    /**
     * @brief Get the length of the encoded fiber-optic TX payload
     *
     * @return Number of uint32_t words (always 2)
     */
    std::uint8_t get_fiber_optic_tx_length() const { return _fiber_optic_tx_length; }

    /**
     * @brief Decode machine-side fiber-optic message and update internal signals
     *
     * Receives a 4-word (16 byte) fiber-optic message from machine-side, decodes it,
     * and updates the external_ios with speed, current reference, voltage magnitude,
     * and force commutation request.
     *
     * @param[in] payload Source buffer containing encoded message
     * @param[in] payload_length Number of uint32_t words in payload
     * @return true if decode successful and signals updated, false otherwise
     */
    bool decode_fiber_optic_rx_from_machine(const std::uint32_t* payload, std::uint8_t payload_length);

    // ==================== External I/O Interface ====================

    /**
     * @brief Get external I/O datamodel for parameter-based signal injection
     *
     * This provides the abstraction layer for all external signals:
     * - Supervisory digital inputs (auxiliary commands, state machine commands)
     * - ADC analog inputs (voltage/current measurements)
     * - Fiber-optics outputs to machine-side (operating mode, current controller saturation, etc.)
     * - Fiber-optics inputs from machine-side (speed, current reference, voltage, etc.)
     *
     * **Simulation Usage:**
     * Plant builder calls update_*() methods to push signals through this datamodel.
     * Machine-side reads from get_fiber_optics_outputs_to_machine().
     *
     * **Test Usage:**
     * Tests can enable override flags and set parameter values to inject mock signals
     * without modifying plant builder or component logic.
     *
     * @see sfc_grid_side_external_ios_datamodel
     */
    sfc_grid_side_external_ios_datamodel& get_external_ios() { return external_ios; }
    const sfc_grid_side_external_ios_datamodel& get_external_ios() const { return external_ios; }

    // ==================== Component Access (for advanced configuration/debugging) ====================

    sfc_state_machine_runner& get_state_machine() { return state_machine; }
    const sfc_state_machine_runner& get_state_machine() const { return state_machine; }

    sfc_current_controller_runner& get_current_controller() { return current_controller; }
    const sfc_current_controller_runner& get_current_controller() const { return current_controller; }

    sfc_grid_side_observer_runner& get_grid_side_observer() { return grid_side_observer; }
    const sfc_grid_side_observer_runner& get_grid_side_observer() const { return grid_side_observer; }

    sfc_grid_side_modulator_runner& get_grid_side_modulator() { return grid_side_modulator; }
    const sfc_grid_side_modulator_runner& get_grid_side_modulator() const { return grid_side_modulator; }

    sfc_grid_side_pulse_generator_runner& get_grid_side_pulse_generator() { return grid_side_pulse_generator; }
    const sfc_grid_side_pulse_generator_runner& get_grid_side_pulse_generator() const
    {
        return grid_side_pulse_generator;
    }

    sfc_grid_side_measurements_runner& get_grid_measurements() { return grid_measurements; }
    const sfc_grid_side_measurements_runner& get_grid_measurements() const { return grid_measurements; }

    sfc_dc_link_measurements_runner& get_dc_link_measurements() { return dc_link_measurements; }
    const sfc_dc_link_measurements_runner& get_dc_link_measurements() const { return dc_link_measurements; }

    sfc_operation_mode_monitor_runner& get_operation_mode_monitor() { return operation_mode_monitor; }
    const sfc_operation_mode_monitor_runner& get_operation_mode_monitor() const { return operation_mode_monitor; }

    sfc_grid_side_integrator_supervisor_runner& get_grid_integrator_supervisor() { return grid_integrator_supervisor; }
    const sfc_grid_side_integrator_supervisor_runner& get_grid_integrator_supervisor() const
    {
        return grid_integrator_supervisor;
    }

    // ==================== External Output Signals ====================

    /**
     * @brief Get operating mode determined by monitor
     */
    const signal_output<sfc_operating_mode_t>& get_operating_mode() const
    {
        return const_cast<sfc_operation_mode_monitor_runner&>(operation_mode_monitor).get_outputs().operating_mode;
    }

    /**
     * @brief Get current zero status from operation mode monitor
     */
    const signal_output<bool>& get_is_current_zero() const
    {
        return const_cast<sfc_operation_mode_monitor_runner&>(operation_mode_monitor).get_outputs().current_is_zero;
    }

    /**
     * @brief Get grid-side firing pulses for plant model
     */
    signal_output<control::six_pulse_generator::pulses>& get_firing_pulses()
    {
        return grid_side_pulse_generator.get_outputs().firing_pulses;
    }

    const signal_output<control::six_pulse_generator::pulses>& get_firing_pulses() const
    {
        return const_cast<sfc_grid_side_pulse_generator_runner&>(grid_side_pulse_generator).get_outputs().firing_pulses;
    }

    /**
     * @brief Get force commutation acknowledgment for machine side
     */
    const signal_output<bool>& get_force_commutation_acknowledged() const
    {
        return const_cast<sfc_grid_side_modulator_runner&>(grid_side_modulator)
            .get_outputs()
            .force_commutation_acknowledged;
    }

    /**
     * @brief Get pulse release status from state machine
     */
    const signal_output<bool>& get_pulse_releases() const
    {
        return const_cast<sfc_state_machine_runner&>(state_machine).get_outputs().release_pulses;
    }

    /**
     * @brief Get freeze integrator signal for cascaded control
     */
    const signal_output<bool>& get_freeze_integrator() const
    {
        return const_cast<sfc_state_machine_runner&>(state_machine).get_outputs().preset_request;
    }

    /**
     * @brief Get current controller saturation status
     */
    const signal_output<bool>& get_current_controller_is_saturated() const
    {
        return const_cast<sfc_current_controller_runner&>(current_controller).get_outputs().is_saturated;
    }

    // ==================== Public Component Access for Direct Signal Connections ====================

    std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system{// Standard per-unit system for SFC applications
        // Grid side: 2090V RMS, 50Hz, DC link: 1044A, Machine: 1980V RMS, 2950kW
        std::make_shared<yeet::application::sfc::sfc_per_unit_system>(
            yeet::voltage_t(2090.0),  // Grid line-to-line voltage (V RMS)
            yeet::frequency_t(50.0),  // Grid frequency (Hz)
            yeet::current_t(1044.0),  // DC link current base (A)
            yeet::power_t(2950e3),    // Rated power (W)
            yeet::voltage_t(1980.0),  // Machine line-to-line voltage (V RMS)
            yeet::frequency_t(50.0),  // Machine base frequency (Hz)
            2                         // Number of pole pairs
            )};

    sfc_per_unit_system_datamodel pu_system_datamodel;

    // Grid-side external I/O datamodel (abstraction layer for all external signals)
    sfc_grid_side_external_ios_datamodel external_ios;

    // Grid-side control components (made public for intermediate integration)
    sfc_state_machine_runner state_machine;
    sfc_grid_side_integrator_supervisor_runner grid_integrator_supervisor;
    sfc_current_controller_runner current_controller;
    sfc_grid_side_observer_runner grid_side_observer;
    sfc_grid_side_modulator_runner grid_side_modulator;
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_grid_side_measurements_runner grid_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;
    sfc_operation_mode_monitor_runner operation_mode_monitor;

private:
    // Fiber-optic communication buffers (internal storage)
    std::array<std::uint32_t, fiber_optic::max_payload_words> _fiber_optic_tx_payload{};
    std::uint8_t _fiber_optic_tx_length{0};

    /**
     * @brief Connect internal signals between grid-side components
     */
    void connect_internal_signals();
};

}  // namespace yeet::application::sfc::integration