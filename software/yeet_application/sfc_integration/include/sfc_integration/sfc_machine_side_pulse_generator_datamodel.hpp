#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
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
        /// Reference angle from machine voltage source (typically from speed/frequency control)
        /// Range: [0.0, 2π] radians
        signal_input<angle_wrapped> reference_angle;

        /// Firing angle command (alpha) in radians
        /// Range: [0.0, π] radians
        signal_input<angle_wrapped> firing_angle;

        signal_input<bool> release_pulses;
    };

    /**
     * @brief Output signals from the machine-side pulse generator
     */
    struct outputs {
        /// Generated firing pulses for the 6-pulse bridge
        /// Contains the state of all 6 thyristor firing signals
        signal_output<control::six_pulse_generator::pulses> firing_pulses;

        /// Actual firing angle being used (for monitoring/debugging)
        /// Range: [0.0, π] radians
        signal_output<angle_wrapped> actual_firing_angle;
    };

    /**
     * @brief Configuration parameters for the machine-side pulse generator
     */
    struct parameters {
        parameter<bool> enable{true};

        /// Pulse width for each firing pulse in radians
        /// Default: π/6 rad (30°)
        parameter<angle_wrapped> pulse_width = angle_wrapped(0.5236);

        /// Reference type (line or phase voltage reference)
        /// Default: phase voltage reference
        parameter<control::six_pulse_generator::reference_type> reference_type =
            control::six_pulse_generator::reference_type::phase;
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
        // Note: Commented out due to compilation issues with signal registry
        // TODO: Fix signal registry insert method compatibility

        // model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // // Register input signals
        // entry.signal_registry.insert("reference_angle", _model_inputs->reference_angle);
        // entry.signal_registry.insert("firing_angle", _model_inputs->firing_angle);
        // entry.signal_registry.insert("enable", _model_inputs->enable);

        // // Register output signals
        // entry.signal_registry.insert("firing_pulses", _model_outputs->firing_pulses);
        // entry.signal_registry.insert("actual_firing_angle", _model_outputs->actual_firing_angle);

        // // Register parameters
        // entry.parameter_registry.insert("min_firing_angle", _model_parameters->min_firing_angle);
        // entry.parameter_registry.insert("max_firing_angle", _model_parameters->max_firing_angle);
        // entry.parameter_registry.insert("pulse_width", _model_parameters->pulse_width);
        // entry.parameter_registry.insert("reference_type", _model_parameters->reference_type);

        // registry.insert("sfc_machine_side_pulse_generator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
