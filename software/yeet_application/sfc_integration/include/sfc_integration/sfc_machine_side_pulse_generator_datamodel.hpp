#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/six_pulse_generator.hpp"

/**
 * @file sfc_machine_side_pulse_generator_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Pulse Generator component
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_pulse_generator_datamodel {
public:
    /**
     * @brief Input signals for the machine-side pulse generator
     */
    struct inputs {
        /// Reference angle from machine control [rad]
        signal_input<angle_wrapped> reference_angle;

        /// Firing angle command (alpha) [rad]
        signal_input<angle_wrapped> firing_angle;

        /// Pulse release control (blocks all pulses when false) [bool]
        signal_input<bool> release_pulses;
    };

    /**
     * @brief Output signals from the machine-side pulse generator
     */
    struct outputs {
        /// Generated firing pulses for 6-pulse bridge
        signal_output<control::six_pulse_generator::pulses> firing_pulses;

        /// Actual firing angle being used [rad]
        signal_output<angle_wrapped> actual_firing_angle;

        /// Pulse generator active status [bool]
        signal_output<bool> release_pulses;

        /// Virtual reference mode active indicator [bool]
        signal_output<bool> virtual_reference_active;

        /// Virtual reference angle [rad]
        signal_output<angle_wrapped> virtual_reference_angle;

        /// Individual pulse override mode active indicator [bool]
        signal_output<bool> individual_override_active;
    };

    /**
     * @brief Configuration parameters for the machine-side pulse generator
     */
    struct parameters {
        /// Master enable override [bool]
        parameter<bool> enable{true};

        /// Pulse width for each firing pulse [rad]
        parameter<angle_wrapped> pulse_width = angle_wrapped::from_degrees(60.0f);

        /// Reference voltage type (line or phase)
        parameter<control::six_pulse_generator::reference_type> reference_type =
            control::six_pulse_generator::reference_type::phase;

        /// Enable virtual reference generator mode [bool]
        parameter<bool> enable_virtual_reference{false};

        /// Virtual reference frequency [Hz]
        parameter<frequency_t> virtual_reference_frequency = frequency_t{50.0};

        /// Enable individual pulse override mode [bool]
        parameter<bool> enable_individual_pulse_override{false};

        /// Individual pulse overrides [bool]
        parameter<bool> override_p1{false};
        parameter<bool> override_p2{false};
        parameter<bool> override_p3{false};
        parameter<bool> override_p4{false};
        parameter<bool> override_p5{false};
        parameter<bool> override_p6{false};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_pulse_generator_datamodel(
        inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // bool parameter - use make_bool_accessor
        entry.parameters.insert("enable", yeet::utils::make_bool_accessor(_model_parameters->enable));

        // angle_wrapped parameter - use make_angle_accessor
        entry.parameters.insert("pulse_width", yeet::utils::make_angle_accessor(_model_parameters->pulse_width));

        // Enum parameter - use make_enum_accessor
        entry.parameters.insert("reference_type", yeet::utils::make_enum_accessor(_model_parameters->reference_type));

        // Virtual reference mode parameters
        entry.parameters.insert(
            "enable_virtual_reference", yeet::utils::make_bool_accessor(_model_parameters->enable_virtual_reference));
        entry.parameters.insert("virtual_reference_frequency",
            yeet::utils::make_units_accessor(_model_parameters->virtual_reference_frequency));

        // Individual pulse override mode parameters
        entry.parameters.insert("enable_individual_pulse_override",
            yeet::utils::make_bool_accessor(_model_parameters->enable_individual_pulse_override));
        entry.parameters.insert("override_p1", yeet::utils::make_bool_accessor(_model_parameters->override_p1));
        entry.parameters.insert("override_p2", yeet::utils::make_bool_accessor(_model_parameters->override_p2));
        entry.parameters.insert("override_p3", yeet::utils::make_bool_accessor(_model_parameters->override_p3));
        entry.parameters.insert("override_p4", yeet::utils::make_bool_accessor(_model_parameters->override_p4));
        entry.parameters.insert("override_p5", yeet::utils::make_bool_accessor(_model_parameters->override_p5));
        entry.parameters.insert("override_p6", yeet::utils::make_bool_accessor(_model_parameters->override_p6));

        registry.insert("pulse_generator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
