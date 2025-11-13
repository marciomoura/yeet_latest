#pragma once

#include "sfc/sfc_startup_sequencer.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_startup_sequencer_datamodel.hpp
 * @brief Datamodel specification for SFC Startup Sequencer component
 *
 * The startup sequencer manages the complete machine-side startup sequence from
 * standstill to normal closed-loop speed control operation. This includes flux
 * build-up, virtual reference frame generation, and transition to observer-based
 * closed-loop control.
 */

namespace yeet::application::sfc::integration {

class sfc_startup_sequencer_datamodel {
public:
    /**
     * @brief Input signals for the startup sequencer component
     */
    struct inputs {
        /// Field excitation established flag from excitation controller
        signal_input<bool> is_field_established;

        /// Rotor angle estimate from machine-side observer [rad]
        signal_input<angle_wrapped> estimated_rotor_angle;

        /// Measured rotor speed from machine-side observer [pu]
        signal_input<speed_pu_t> measured_speed;

        /// Measured voltage magnitude from machine-side observer [pu]
        signal_input<voltage_pu_t> machine_voltage_magnitude;

        /// Grid-side state machine current state ID
        signal_input<yeet::application::sfc::sfc_state_id> current_state_id;

        /// Start command signal (manual override)
        signal_input<bool> start_command;

        /// Stop command signal (manual override)
        signal_input<bool> stop_command;

        /// External speed reference [pu]
        signal_input<speed_pu_t> external_speed_reference;
    };

    /**
     * @brief Output signals from the startup sequencer component
     */
    struct outputs {
        /// Current state of the startup sequence
        signal_output<startup_sequencer_state_t> current_state;

        /// Reference frame source (VIRTUAL or ESTIMATED)
        signal_output<reference_frame_source_t> reference_frame_source;

        /// Torque control mode (SPEED_CONTROL or MANUAL_TORQUE)
        signal_output<sfc_torque_controller::control_mode> torque_control_mode;

        /// Enable signal for machine-side pulse generation
        signal_output<bool> release_machine_side_pulses;

        /// Enable signal for excitation controller
        signal_output<bool> release_excitation;

        /// Enable signal for machine observer
        signal_output<bool> release_machine_observer;

        /// Enable signal for speed controller
        signal_output<bool> release_speed_controller;

        /// Enable signal for torque controller
        signal_output<bool> release_torque_controller;

        /// Virtual rotor angle [rad]
        signal_output<angle_wrapped> virtual_rotor_angle;

        /// Virtual rotor frequency [Hz]
        signal_output<frequency_pu_t> virtual_rotor_frequency;

        /// Virtual torque reference [pu]
        signal_output<torque_pu_t> virtual_torque_reference;

        /// Rate-limited speed reference [pu]
        signal_output<speed_pu_t> speed_reference;

        /// Startup complete status (true when in CLOSED_LOOP_OPERATION)
        signal_output<bool> is_startup_complete;

        /// Time elapsed in current state [s]
        signal_output<duration_t> time_in_current_state;
    };

    /**
     * @brief Configuration parameters for the startup sequencer
     */
    struct parameters {
        /// Initial virtual frequency [pu]
        parameter<frequency_pu_t> virtual_frequency_initial = frequency_pu_t{0.0};

        /// Final virtual frequency [pu]
        parameter<frequency_pu_t> virtual_frequency_final = frequency_pu_t{0.2};

        /// Virtual frequency ramp duration [s], 0 = skip virtual phase
        parameter<duration_t> virtual_frequency_duration = duration_t{10.0};

        /// Initial virtual torque reference [pu]
        parameter<torque_pu_t> virtual_torque_initial = torque_pu_t{0.3};

        /// Final virtual torque reference [pu]
        parameter<torque_pu_t> virtual_torque_final = torque_pu_t{0.3};

        /// Virtual torque ramp duration [s], 0 = immediate jump to final
        parameter<duration_t> virtual_torque_duration = duration_t{0.0};

        /// Enable override for start command testing
        parameter<bool> override_enable_start_command = false;

        /// Forced start command value when override is enabled
        parameter<bool> override_value_start_command = false;

        /// Enable override for stop command testing
        parameter<bool> override_enable_stop_command = false;

        /// Forced stop command value when override is enabled
        parameter<bool> override_value_stop_command = false;

        /// Enable override for field established flag testing
        parameter<bool> override_enable_field_established = false;

        /// Forced field established value when override is enabled
        parameter<bool> override_value_field_established = false;

        /// Enable override for external speed reference testing
        parameter<bool> override_enable_speed_reference = false;

        /// Forced speed reference value when override is enabled [pu]
        parameter<speed_pu_t> override_value_speed_reference = speed_pu_t{0.0};

        /// Speed reference rate limit in seconds per per-unit speed change
        /// (e.g., 10.0 means 10 seconds to ramp from 0 to 1.0 pu = 0.1 pu/s)
        parameter<real_t> speed_reference_rate_limit_seconds_per_pu = real_t{10.0};

        /// Enable/disable speed reference rate limiting
        parameter<bool> speed_reference_rate_limit_enabled = true;

        /// Minimum speed threshold for flying restart [pu]
        /// (e.g., 0.05 = 5% rated speed)
        parameter<speed_pu_t> flying_restart_speed_threshold = speed_pu_t{0.05};

        /// Voltage magnitude threshold for flying restart detection [pu]
        /// (e.g., 0.1 = 10% rated voltage)
        parameter<voltage_pu_t> flying_restart_voltage_threshold = voltage_pu_t{0.1};

        /// Observer settling time before flying restart decision [s]
        parameter<duration_t> flying_restart_observer_settling_time = duration_t{2.0};

        /// Enable automatic start/stop based on state machine state
        parameter<bool> auto_start_stop_enabled = true;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_startup_sequencer_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register parameters
        entry.parameters.insert("virtual_frequency_initial",
            yeet::utils::make_units_accessor(_model_parameters->virtual_frequency_initial));
        entry.parameters.insert(
            "virtual_frequency_final", yeet::utils::make_units_accessor(_model_parameters->virtual_frequency_final));
        entry.parameters.insert("virtual_frequency_duration",
            yeet::utils::make_units_accessor(_model_parameters->virtual_frequency_duration));
        entry.parameters.insert(
            "virtual_torque_initial", yeet::utils::make_units_accessor(_model_parameters->virtual_torque_initial));
        entry.parameters.insert(
            "virtual_torque_final", yeet::utils::make_units_accessor(_model_parameters->virtual_torque_final));
        entry.parameters.insert(
            "virtual_torque_duration", yeet::utils::make_units_accessor(_model_parameters->virtual_torque_duration));
        entry.parameters.insert("override_enable_start_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_start_command));
        entry.parameters.insert("override_value_start_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_start_command));
        entry.parameters.insert("override_enable_stop_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_stop_command));
        entry.parameters.insert("override_value_stop_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_stop_command));
        entry.parameters.insert("override_enable_field_established",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_field_established));
        entry.parameters.insert("override_value_field_established",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_field_established));
        entry.parameters.insert("override_enable_speed_reference",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_speed_reference));
        entry.parameters.insert("override_value_speed_reference",
            yeet::utils::make_units_accessor(_model_parameters->override_value_speed_reference));
        entry.parameters.insert("speed_reference_rate_limit_seconds_per_pu",
            yeet::utils::make_units_accessor(_model_parameters->speed_reference_rate_limit_seconds_per_pu));
        entry.parameters.insert("speed_reference_rate_limit_enabled",
            yeet::utils::make_bool_accessor(_model_parameters->speed_reference_rate_limit_enabled));
        entry.parameters.insert("flying_restart_speed_threshold",
            yeet::utils::make_units_accessor(_model_parameters->flying_restart_speed_threshold));
        entry.parameters.insert("flying_restart_voltage_threshold",
            yeet::utils::make_units_accessor(_model_parameters->flying_restart_voltage_threshold));
        entry.parameters.insert("flying_restart_observer_settling_time",
            yeet::utils::make_units_accessor(_model_parameters->flying_restart_observer_settling_time));
        entry.parameters.insert(
            "auto_start_stop_enabled", yeet::utils::make_bool_accessor(_model_parameters->auto_start_stop_enabled));

        registry.insert("startup_sequencer", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
