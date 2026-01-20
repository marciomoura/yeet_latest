#pragma once

#include "sfc/sfc_grid_side_modulator.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_grid_side_modulator_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Modulator component
 *
 * The grid-side modulator acts as the master controller for forced commutation,
 * coordinating the handshake protocol with the machine-side modulator and
 * managing the override of the current controller's firing angle during
 * commutation events.
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_modulator_datamodel {
public:
    /**
     * @brief Input signals for the grid-side modulator component
     */
    struct inputs {
        /// Request signal from machine-side modulator to initiate forced commutation [bool]
        signal_input<bool> request_force_commutation;

        /// Request signal to extinguish dc-link current [bool]
        signal_input<bool> request_extinguish_dc_current;

        /// Signal indicating DC link current is zero [bool]
        signal_input<bool> current_is_zero;

        /// Firing angle reference from current controller [rad]
        signal_input<angle_wrapped> firing_angle;

        /// Operating mode selector (natural vs pulse mode)
        signal_input<sfc_operating_mode_t> operating_mode;

        /// External shutdown/trip signal [bool]
        signal_input<bool> shutdown_request;

        /// Fast protection trip signal (direct from protection system) [bool]
        signal_input<bool> protection_trip;

        /// Release pulses control signal from integrator supervisor [bool]
        signal_input<bool> release_pulses;

        /// Grid reference angle from PLL [rad]
        signal_input<angle_wrapped> grid_reference_angle;
    };

    /**
     * @brief Output signals from the grid-side modulator component
     */
    struct outputs {
        /// Pulse release control signal to pulse generator [bool]
        signal_output<bool> release_pulses;

        /// Firing angle output to pulse generator [rad]
        signal_output<angle_wrapped> firing_angle;

        /// Acknowledgment signal to machine-side modulator [bool]
        signal_output<bool> force_commutation_acknowledged;

        /// Request signal to freeze current controller integrator [bool]
        signal_output<bool> freeze_integrator;

        /// Current modulator state for diagnostics
        signal_output<grid_side_modulator_state_t> modulator_state;

        /// Firing pulses for the thyristors
        signal_output<sfc_thyristor_pulses_t> firing_pulses;

        /// Indicates if any thyristor has been fired this cycle
        signal_output<bool> any_thyristor_fired;

        /// Internally integrated virtual reference angle [rad]
        signal_output<angle_wrapped> virtual_reference_angle;

        /// Indicates if virtual reference mode is active [bool]
        signal_output<bool> virtual_reference_active;

        /// Indicates if individual pulse override is active [bool]
        signal_output<bool> individual_override_active;
    };

    /**
     * @brief Configuration parameters for the grid-side modulator
     */
    struct parameters {
        /// Master enable override [bool]
        parameter<bool> enable{true};

        /// Pulse width for each firing pulse [rad]
        parameter<angle_wrapped> pulse_width = angle_wrapped::from_degrees(120.0f);

        /// Reference voltage type (line or phase)
        parameter<control::six_pulse_generator::reference_type> reference_type =
            control::six_pulse_generator::reference_type::phase;

        /// Firing angle for forced commutation [rad]
        parameter<angle_wrapped> force_commutation_angle = angle_wrapped(2.6180);

        /// Timeout duration for forced commutation [s]
        parameter<duration_t> commutation_timeout = duration_t(0.1);

        /// Enable open-loop firing angle override [bool]
        parameter<bool> force_open_loop_firing_angle = false;

        /// Value for open-loop firing angle override [rad]
        parameter<angle_wrapped> open_loop_firing_angle_value = angle_wrapped::from_degrees(90.0f);

        /// Frequency for virtual reference generation [Hz]
        parameter<frequency_t> virtual_reference_frequency = frequency_t(0.0);

        /// Enable flag for virtual reference mode
        parameter<bool> enable_virtual_reference = false;

        /// Enabled flag for individual pulse override
        parameter<bool> enable_individual_pulse_override = false;

        /// Override signals for individual pulses
        parameter<bool> override_p1 = true;
        parameter<bool> override_p2 = true;
        parameter<bool> override_p3 = true;
        parameter<bool> override_p4 = true;
        parameter<bool> override_p5 = true;
        parameter<bool> override_p6 = true;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_modulator_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "grid_side_modulator";

        // Parameters
        handler.register_parameter(
            component_name, "enable", yeet::utils::make_bool_accessor(_model_parameters->enable));
        handler.register_parameter(component_name, "pulse_width",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->pulse_width));
        handler.register_parameter(
            component_name, "reference_type", yeet::utils::make_enum_accessor(_model_parameters->reference_type));
        handler.register_parameter(component_name, "force_commutation_angle",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->force_commutation_angle));
        handler.register_parameter(component_name, "commutation_timeout",
            yeet::utils::make_units_accessor(_model_parameters->commutation_timeout));
        handler.register_parameter(component_name, "force_open_loop_firing_angle",
            yeet::utils::make_bool_accessor(_model_parameters->force_open_loop_firing_angle));
        handler.register_parameter(component_name, "open_loop_firing_angle_value",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->open_loop_firing_angle_value));
        handler.register_parameter(component_name, "virtual_reference_frequency",
            yeet::utils::make_units_accessor(_model_parameters->virtual_reference_frequency));
        handler.register_parameter(component_name, "enable_virtual_reference",
            yeet::utils::make_bool_accessor(_model_parameters->enable_virtual_reference));
        handler.register_parameter(component_name, "enable_individual_pulse_override",
            yeet::utils::make_bool_accessor(_model_parameters->enable_individual_pulse_override));
        handler.register_parameter(
            component_name, "override_p1", yeet::utils::make_bool_accessor(_model_parameters->override_p1));
        handler.register_parameter(
            component_name, "override_p2", yeet::utils::make_bool_accessor(_model_parameters->override_p2));
        handler.register_parameter(
            component_name, "override_p3", yeet::utils::make_bool_accessor(_model_parameters->override_p3));
        handler.register_parameter(
            component_name, "override_p4", yeet::utils::make_bool_accessor(_model_parameters->override_p4));
        handler.register_parameter(
            component_name, "override_p5", yeet::utils::make_bool_accessor(_model_parameters->override_p5));
        handler.register_parameter(
            component_name, "override_p6", yeet::utils::make_bool_accessor(_model_parameters->override_p6));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
