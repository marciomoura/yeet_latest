#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"
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
 */
class sfc_grid_side_control {
public:
    /**
     * @brief Construct grid-side control with per-unit system
     * @param per_unit_system Shared per-unit system for scaling
     */
    explicit sfc_grid_side_control(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
        : _per_unit_system(per_unit_system),
          state_machine(),
          current_controller(_per_unit_system),
          grid_side_observer(),
          grid_side_modulator(),
          grid_side_pulse_generator(),
          grid_measurements(_per_unit_system),
          dc_link_measurements(_per_unit_system),
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

    // ==================== Component Access ====================

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

    // ==================== External Input Signals ====================

    // State machine control inputs
    signal_output<bool> auxiliaries_are_on;
    signal_output<bool> breakers_are_on;
    signal_output<bool> start_command;
    signal_output<bool> stop_command;
    signal_output<bool> shutdown_request;
    signal_output<bool> is_current_extinguished;

    // Current controller reference
    // signal_output<current_pu_t> dc_link_current_reference;

    // Speed feedback for operation mode monitor
    // signal_output<speed_pu_t> measured_speed_pu;

    // Force commutation request from machine side
    // signal_output<bool> request_force_commutation;

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
        return const_cast<sfc_operation_mode_monitor_runner&>(operation_mode_monitor).get_outputs().is_current_zero;
    }

    /**
     * @brief Get grid-side firing pulses for plant model
     */
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
        return const_cast<sfc_state_machine_runner&>(state_machine).get_outputs().pulse_releases;
    }

    /**
     * @brief Get freeze integrator signal for cascaded control
     */
    const signal_output<bool>& get_freeze_integrator() const
    {
        return const_cast<sfc_state_machine_runner&>(state_machine).get_outputs().freeze_integrator;
    }

    /**
     * @brief Get current controller saturation status
     */
    const signal_output<bool>& get_current_controller_is_saturated() const
    {
        return const_cast<sfc_current_controller_runner&>(current_controller)
            .get_outputs()
            .current_controller_is_saturated;
    }

    // ==================== Public Component Access for Direct Signal Connections ====================

    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Grid-side control components (made public for intermediate integration)
    sfc_state_machine_runner state_machine;
    sfc_current_controller_runner current_controller;
    sfc_grid_side_observer_runner grid_side_observer;
    sfc_grid_side_modulator_runner grid_side_modulator;
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_grid_side_measurements_runner grid_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;
    sfc_operation_mode_monitor_runner operation_mode_monitor;

private:
    /**
     * @brief Connect internal signals between grid-side components
     */
    void connect_internal_signals();
};

}  // namespace yeet::application::sfc::integration