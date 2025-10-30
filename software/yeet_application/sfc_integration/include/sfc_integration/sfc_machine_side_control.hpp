#pragma once

#include "sfc_integration/sfc_excitation_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_control_io.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_per_unit_system_datamodel.hpp"
#include "sfc_integration/sfc_soft_starter_speed_generator_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_integration/sfc_torque_controller_runner.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Machine-side control system for Static Frequency Converter
 *
 * This class encapsulates all machine-side control components including:
 * - Excitation controller for machine excitation voltage regulation
 * - Speed controller for machine speed regulation
 * - Machine-side controller for firing angle determination
 * - Machine-side observer for speed/position estimation
 * - Machine-side modulator and pulse generator
 * - Machine measurement processing
 */
class sfc_machine_side_control {
public:
    /**
     * @brief Construct machine-side control with per-unit system
     * @param per_unit_system Shared per-unit system for scaling
     */
    explicit sfc_machine_side_control()
        : excitation_controller(),
          speed_generator(),
          speed_controller(),
          torque_controller(),
          machine_side_controller(),
          machine_side_observer(),
          machine_side_modulator(),
          machine_side_pulse_generator(),
          machine_measurements(per_unit_system),
          io(machine_measurements,
              excitation_controller,
              speed_generator,
              speed_controller,
              torque_controller,
              machine_side_controller,
              machine_side_observer,
              machine_side_modulator,
              machine_side_pulse_generator)
    {
        connect_internal_signals();
    }

    /**
     * @brief Initialize all machine-side components
     */
    void initialize();

    /**
     * @brief Update components running at 10ms sampling time (100 Hz)
     *
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
     * @brief Register all component datamodels with the model registry
     *
     * This registers all parameters from grid-side components (current controller,
     * observer, modulator, pulse generator, measurements, state machine, operation mode monitor)
     * with the provided registry, making them accessible via parameter configuration protocol.
     *
     * @param registry The model registry to populate with component parameters
     */
    void register_datamodel(yeet::model_registry& registry);

    // ==================== Component Access ====================

    sfc_excitation_controller_runner& get_excitation_controller() { return excitation_controller; }
    const sfc_excitation_controller_runner& get_excitation_controller() const { return excitation_controller; }

    sfc_soft_starter_speed_generator_runner& get_speed_generator() { return speed_generator; }
    const sfc_soft_starter_speed_generator_runner& get_speed_generator() const { return speed_generator; }

    sfc_speed_controller_runner& get_speed_controller() { return speed_controller; }
    const sfc_speed_controller_runner& get_speed_controller() const { return speed_controller; }

    sfc_torque_controller_runner& get_torque_controller() { return torque_controller; }
    const sfc_torque_controller_runner& get_torque_controller() const { return torque_controller; }

    sfc_machine_side_controller_runner& get_machine_side_controller() { return machine_side_controller; }
    const sfc_machine_side_controller_runner& get_machine_side_controller() const { return machine_side_controller; }

    sfc_machine_side_observer_runner& get_machine_side_observer() { return machine_side_observer; }
    const sfc_machine_side_observer_runner& get_machine_side_observer() const { return machine_side_observer; }

    sfc_machine_side_modulator_runner& get_machine_side_modulator() { return machine_side_modulator; }
    const sfc_machine_side_modulator_runner& get_machine_side_modulator() const { return machine_side_modulator; }

    sfc_machine_side_pulse_generator_runner& get_machine_side_pulse_generator() { return machine_side_pulse_generator; }
    const sfc_machine_side_pulse_generator_runner& get_machine_side_pulse_generator() const
    {
        return machine_side_pulse_generator;
    }

    sfc_machine_side_measurements_runner& get_machine_measurements() { return machine_measurements; }
    const sfc_machine_side_measurements_runner& get_machine_measurements() const { return machine_measurements; }

    /**
     * @brief Get I/O interface for external connections
     */
    sfc_machine_side_control_io& get_io() { return io; }
    const sfc_machine_side_control_io& get_io() const { return io; }

    // ==================== External Input Signals (DEPRECATED - Use io interface) ====================

    // Speed generator profile commands (semantic names for turbine starting sequence)
    signal_output<bool> run_creep_speed;     // Execute Profile 1: Creep speed for maintenance (~5% rated)
    signal_output<bool> run_purge_accel;     // Execute Profile 2: Purge acceleration (~20% rated)
    signal_output<bool> run_ignition_accel;  // Execute Profile 3: Ignition acceleration (~55% rated)
    signal_output<bool> run_assist_accel;    // Execute Profile 4: Assist to self-sustaining (~95% rated)
    signal_output<bool> hold_speed;          // Hold current speed setpoint
    signal_output<bool> decelerate;          // Controlled deceleration to zero
    signal_output<bool> idle;                // Idle mode (zero torque/speed outputs)

    // Excitation control inputs
    signal_output<bool> exciter_ready;
    signal_output<bool> excitation_start_command;
    signal_output<bool> excitation_stop_command;

    // Speed controller cascade inputs from grid-side
    signal_output<bool> enable_speed_control;         // From grid-side pulse_releases
    signal_output<bool> freeze_integrator;            // From grid-side machine-side outputs
    signal_output<bool> modulator_freeze_integrator;  // From grid-side modulator pi reset

    // Cascaded control signals from grid side
    // signal_output<bool> freeze_integrator;
    // signal_output<bool> current_controller_is_saturated;
    // signal_output<bool> pulse_releases;

    // Operating mode from grid side
    // signal_output<sfc_operating_mode_t> operating_mode;

    // DC link current for machine controller lookup
    // signal_output<current_pu_t> dc_link_current_pu;

    // Force commutation acknowledgment from grid side
    // signal_output<bool> force_commutation_acknowledged;

    // Machine-side voltage magnitude for current controller feedforward
    // signal_output<voltage_pu_t> machine_side_voltage_magnitude;

    // ==================== External Output Signals ====================

    /**
     * @brief Get excitation voltage setpoint from excitation controller
     */
    const signal_output<voltage_pu_t>& get_excitation_voltage_setpoint() const
    {
        return const_cast<sfc_excitation_controller_runner&>(excitation_controller)
            .get_outputs()
            .excitation_voltage_setpoint_pu;
    }

    /**
     * @brief Get excitation enable command from excitation controller
     */
    const signal_output<bool>& get_enable_excitation() const
    {
        return const_cast<sfc_excitation_controller_runner&>(excitation_controller).get_outputs().enable_excitation;
    }

    /**
     * @brief Get field established status from excitation controller
     */
    const signal_output<bool>& get_is_field_established() const
    {
        return const_cast<sfc_excitation_controller_runner&>(excitation_controller).get_outputs().is_field_established;
    }

    /**
     * @brief Get machine-side firing pulses for plant model
     */
    const signal_output<control::six_pulse_generator::pulses>& get_firing_pulses() const
    {
        return const_cast<sfc_machine_side_pulse_generator_runner&>(machine_side_pulse_generator)
            .get_outputs()
            .firing_pulses;
    }

    /**
     * @brief Get estimated machine speed from observer
     */
    const signal_output<speed_pu_t>& get_estimated_speed() const
    {
        return const_cast<sfc_machine_side_observer_runner&>(machine_side_observer).get_outputs().estimated_speed;
    }

    /**
     * @brief Get estimated machine position from observer
     */
    const signal_output<angle_wrapped>& get_estimated_position() const
    {
        return const_cast<sfc_machine_side_observer_runner&>(machine_side_observer).get_outputs().estimated_position;
    }

    /**
     * @brief Get force commutation request for grid side
     */
    const signal_output<bool>& get_request_force_commutation() const
    {
        return const_cast<sfc_machine_side_modulator_runner&>(machine_side_modulator)
            .get_outputs()
            .request_force_commutation;
    }

    /**
     * @brief Get current reference from torque controller
     */
    const signal_output<current_pu_t>& get_current_reference() const
    {
        return const_cast<sfc_torque_controller_runner&>(torque_controller).get_outputs().current_reference;
    }

    /**
     * @brief Get estimated voltage magnitude from observer
     */
    const signal_output<voltage_pu_t>& get_estimated_voltage_magnitude() const
    {
        return const_cast<sfc_machine_side_observer_runner&>(machine_side_observer)
            .get_outputs()
            .estimated_voltage_magnitude;
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

    // Machine-side control components (made public for intermediate integration)
    sfc_excitation_controller_runner excitation_controller;
    sfc_soft_starter_speed_generator_runner speed_generator;
    sfc_speed_controller_runner speed_controller;
    sfc_torque_controller_runner torque_controller;
    sfc_machine_side_controller_runner machine_side_controller;
    sfc_machine_side_observer_runner machine_side_observer;
    sfc_machine_side_modulator_runner machine_side_modulator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;
    sfc_machine_side_measurements_runner machine_measurements;

    // I/O interface for external connections (must be last as it references all components)
    sfc_machine_side_control_io io;

private:
    /**
     * @brief Connect internal signals between machine-side components
     */
    void connect_internal_signals();
};

}  // namespace yeet::application::sfc::integration