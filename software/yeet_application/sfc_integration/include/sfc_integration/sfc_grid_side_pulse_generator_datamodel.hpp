#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/six_pulse_generator.hpp"

/**
 * @file sfc_grid_side_pulse_generator_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Pulse Generator component
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_pulse_generator_datamodel {
public:
    /**
     * @brief Input signals for the grid-side pulse generator
     */
    struct inputs {
        /// Reference angle from grid voltage source (typically from PLL)
        /// Range: [0.0, 2π] radians
        signal_input<angle_wrapped> reference_angle;

        /// Firing angle command (alpha) in radians
        /// Range: [0.0, π] radians
        signal_input<angle_wrapped> firing_angle;

        /// Enable/disable signal for pulse generation
        /// true = pulses enabled, false = all pulses disabled
        signal_input<bool> enable;
    };

    /**
     * @brief Output signals from the grid-side pulse generator
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
     * @brief Configuration parameters for the grid-side pulse generator
     */
    struct parameters {
        /// Minimum allowed firing angle in radians
        /// Default: 0.0 rad (0°)
        parameter<angle_wrapped> min_firing_angle = angle_wrapped(0.0);

        /// Maximum allowed firing angle in radians
        /// Default: π/2 rad (90°)
        parameter<angle_wrapped> max_firing_angle = angle_wrapped(1.5708);

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
    sfc_grid_side_pulse_generator_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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

        // registry.insert("sfc_grid_side_pulse_generator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
