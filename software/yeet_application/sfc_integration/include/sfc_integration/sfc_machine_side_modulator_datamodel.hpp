#pragma once

#include <cstdint>

#include "sfc/sfc_machine_side_modulator.hpp"
#include "sfc/sfc_startup_sequencer.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"

/**
 * @file sfc_machine_side_modulator_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Modulator component
 *
 * The machine-side modulator initiates forced commutation events and manages
 * firing angle selection based on operating mode. It coordinates with the
 * grid-side modulator through a handshake protocol for commutation at low speeds.
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_modulator_datamodel {
public:
    /**
     * @brief Input signals for the machine-side modulator component
     */
    struct inputs {
        /// Operating mode selector (natural vs pulse)
        signal_input<sfc_operating_mode_t> operating_mode;

        /// Acknowledgment signal from grid-side modulator [bool]
        signal_input<bool> force_commutation_acknowledged;

        /// Rotor electrical angle from observer for sector tracking [rad]
        signal_input<angle_wrapped> machine_reference_angle;

        /// Virtual rotor angle from startup sequencer [rad]
        signal_input<angle_wrapped> virtual_rotor_angle;

        /// Reference frame source selection (VIRTUAL or ESTIMATED)
        signal_input<reference_frame_source_t> reference_frame_source;

        /// Machine-side firing angle reference [rad]
        signal_input<angle_wrapped> firing_angle;

        /// Pulse release control from supervisor [bool]
        signal_input<bool> release_pulses;
    };

    /**
     * @brief Output signals from the machine-side modulator component
     */
    struct outputs {
        /// Pulse release control signal [bool]
        signal_output<bool> release_pulses;

        /// Firing angle output to pulse generator [rad]
        signal_output<angle_wrapped> firing_angle;

        /// Selected firing angle reference source [rad]
        signal_output<angle_wrapped> selected_firing_angle_reference;

        /// Request signal to grid-side modulator for forced commutation [bool]
        signal_output<bool> request_force_commutation;

        /// Current commutation state for diagnostics
        signal_output<machine_side_commutation_state_t> commutation_state;

        /// Current rotor sector (1-6) for diagnostics
        signal_output<std::uint8_t> current_rotor_sector;

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
     * @brief Configuration parameters for the machine-side modulator
     */
    struct parameters {
        /// Master enable override [bool]
        parameter<bool> enable{true};

        /// Pulse width for each firing pulse [rad]
        parameter<angle_wrapped> pulse_width = angle_wrapped::from_degrees(120.0f);

        /// Reference voltage type (line or phase)
        parameter<control::six_pulse_generator::reference_type> reference_type =
            control::six_pulse_generator::reference_type::phase;

        /// Firing angle for pulse mode operation [rad]
        parameter<angle_wrapped> forced_commutation_firing_angle = angle_wrapped(pi);

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
    sfc_machine_side_modulator_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "machine_side_modulator";

        // Parameters
        handler.register_parameter(
            component_name, "enable", yeet::utils::make_bool_accessor(_model_parameters->enable));
        handler.register_parameter(component_name, "pulse_width",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->pulse_width));
        handler.register_parameter(
            component_name, "reference_type", yeet::utils::make_enum_accessor(_model_parameters->reference_type));
        handler.register_parameter(component_name, "forced_commutation_firing_angle",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->forced_commutation_firing_angle));
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
