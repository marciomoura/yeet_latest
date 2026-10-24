#pragma once

#include "sfc/sfc_startup_sequencer.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
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

        /// Startup active status (true during FLUX_BUILDUP and STARTING_PROCEDURE)
        /// Used to arm state-dependent protections like stall detection.
        signal_output<bool> startup_active;

        /// Time elapsed in current state [s]
        signal_output<duration_t> time_in_current_state;

        /// Preset signal to initialize observer angle integrator
        signal_output<bool> preset_observer;

        /// Preset angle value for observer integrator initialization [rad]
        signal_output<angle_wrapped> preset_angle;

        /// Preset frequency value for observer frequency initialization [pu]
        signal_output<frequency_pu_t> preset_frequency;
    };

    /**
     * @brief Configuration parameters for the startup sequencer
     */
    struct parameters {
        /// Initial virtual frequency [pu]
        parameter<frequency_pu_t> virtual_frequency_initial = frequency_pu_t{0.0};

        /// Final virtual frequency [pu]
        parameter<frequency_pu_t> virtual_frequency_final = frequency_pu_t{0.1};

        /// Virtual frequency ramp duration [s], 0 = skip virtual phase
        parameter<duration_t> virtual_frequency_duration = duration_t{2.0};

        /// Initial virtual torque reference [pu]
        parameter<torque_pu_t> virtual_torque_initial = torque_pu_t{0.0};

        /// Final virtual torque reference [pu]
        parameter<torque_pu_t> virtual_torque_final = torque_pu_t{0.3};

        /// Virtual torque ramp duration [s], 0 = immediate jump to final
        parameter<duration_t> virtual_torque_duration = duration_t{0.5};

        /// Enable damping based on observer speed feedback during virtual mode
        parameter<bool> virtual_torque_damping_enabled = false;

        /// Proportional gain for virtual torque damping (pu_torque / pu_speed_error)
        parameter<real_t> virtual_torque_damping_kp = real_t{1.0};

        /// Integral time constant for virtual torque damping [s]
        parameter<real_t> virtual_torque_damping_ti = real_t{0.5};

        /// Speed threshold above which damping is active [pu]
        parameter<speed_pu_t> virtual_torque_damping_activation_speed = speed_pu_t{0.01};

        /// Skip flux build-up phase (useful for tests without excitation)
        parameter<bool> skip_flux_buildup = false;

        /// Enable override for field established flag testing
        parameter<bool> field_established_override = false;

        /// Forced field established value when override is enabled
        parameter<bool> field_established_override_value = false;

        /// External speed reference command [pu]
        parameter<speed_pu_t> speed_reference = speed_pu_t{1.0};

        /// Speed reference rate limit in seconds per per-unit speed change
        /// (e.g., 10.0 means 10 seconds to ramp from 0 to 1.0 pu = 0.1 pu/s)
        parameter<real_t> speed_reference_rate_limit_seconds_per_pu = real_t{20.0};

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

        /// Enable flying restart detection
        parameter<bool> flying_restart_enabled = false;

        /// Force virtual reference mode operation (for commissioning with short-circuited machine side)
        /// When enabled, the sequencer always uses VIRTUAL reference frame regardless of state.
        /// This is useful during commissioning when the inverter side is short-circuited.
        parameter<bool> force_virtual_mode_operation = false;
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "startup_sequencer";
        // Register parameters
        handler.register_parameter(component_name, "virtual_frequency_initial",
            yeet::utils::make_units_accessor(_model_parameters->virtual_frequency_initial));
        handler.register_parameter(component_name, "virtual_frequency_final",
            yeet::utils::make_units_accessor(_model_parameters->virtual_frequency_final));
        handler.register_parameter(component_name, "virtual_frequency_duration",
            yeet::utils::make_units_accessor(_model_parameters->virtual_frequency_duration));
        handler.register_parameter(component_name, "virtual_torque_initial",
            yeet::utils::make_units_accessor(_model_parameters->virtual_torque_initial));
        handler.register_parameter(component_name, "virtual_torque_final",
            yeet::utils::make_units_accessor(_model_parameters->virtual_torque_final));
        handler.register_parameter(component_name, "virtual_torque_duration",
            yeet::utils::make_units_accessor(_model_parameters->virtual_torque_duration));
        handler.register_parameter(component_name, "virtual_torque_damping_enabled",
            yeet::utils::make_bool_accessor(_model_parameters->virtual_torque_damping_enabled));
        handler.register_parameter(component_name, "virtual_torque_damping_kp",
            yeet::utils::make_units_accessor(_model_parameters->virtual_torque_damping_kp));
        handler.register_parameter(component_name, "virtual_torque_damping_ti",
            yeet::utils::make_units_accessor(_model_parameters->virtual_torque_damping_ti));
        handler.register_parameter(component_name, "virtual_torque_damping_activation_speed",
            yeet::utils::make_units_accessor(_model_parameters->virtual_torque_damping_activation_speed));
        handler.register_parameter(component_name, "field_established_override",
            yeet::utils::make_bool_accessor(_model_parameters->field_established_override));
        handler.register_parameter(component_name, "field_established_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->field_established_override_value));
        handler.register_parameter(
            component_name, "speed_reference", yeet::utils::make_units_accessor(_model_parameters->speed_reference));
        handler.register_parameter(component_name, "speed_reference_rate_limit_seconds_per_pu",
            yeet::utils::make_units_accessor(_model_parameters->speed_reference_rate_limit_seconds_per_pu));
        handler.register_parameter(component_name, "speed_reference_rate_limit_enabled",
            yeet::utils::make_bool_accessor(_model_parameters->speed_reference_rate_limit_enabled));
        handler.register_parameter(component_name, "flying_restart_speed_threshold",
            yeet::utils::make_units_accessor(_model_parameters->flying_restart_speed_threshold));
        handler.register_parameter(component_name, "flying_restart_voltage_threshold",
            yeet::utils::make_units_accessor(_model_parameters->flying_restart_voltage_threshold));
        handler.register_parameter(component_name, "flying_restart_observer_settling_time",
            yeet::utils::make_units_accessor(_model_parameters->flying_restart_observer_settling_time));
        handler.register_parameter(component_name, "flying_restart_enabled",
            yeet::utils::make_bool_accessor(_model_parameters->flying_restart_enabled));
        handler.register_parameter(component_name, "force_virtual_mode_operation",
            yeet::utils::make_bool_accessor(_model_parameters->force_virtual_mode_operation));
        handler.register_parameter(
            component_name, "skip_flux_buildup", yeet::utils::make_bool_accessor(_model_parameters->skip_flux_buildup));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
