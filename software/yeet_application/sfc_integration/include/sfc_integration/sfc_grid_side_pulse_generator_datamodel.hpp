#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/six_pulse_generator.hpp"

/**
 * @file sfc_grid_side_pulse_generator_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Pulse Generator component
 *
 * The pulse generator creates the six firing pulses for the grid-side thyristor
 * bridge based on the reference angle from the PLL and the firing angle from
 * the modulator.
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_pulse_generator_datamodel {
public:
    /**
     * @brief Input signals for the grid-side pulse generator
     */
    struct inputs {
        /// Reference angle from grid voltage observer (PLL output)
        /// Range: [0.0, 2π] radians
        signal_input<angle_wrapped> reference_angle;

        /// Firing angle command from grid-side modulator
        /// Range: [0.0, π] radians
        signal_input<angle_wrapped> firing_angle;

        /// Pulse release control (blocks all pulses when false)
        signal_input<bool> release_pulses;
    };

    /**
     * @brief Output signals from the grid-side pulse generator
     */
    struct outputs {
        /// Generated firing pulses for the 6-pulse bridge
        /// Contains the state of all 6 thyristor firing signals
        signal_output<control::six_pulse_generator::pulses> firing_pulses;

        /// Actual firing angle being applied (pass-through for monitoring)
        /// Range: [0.0, π] radians
        signal_output<angle_wrapped> actual_firing_angle;

        /// Pulse generator active status
        signal_output<bool> release_pulses;
    };

    /**
     * @brief Configuration parameters for the grid-side pulse generator
     */
    struct parameters {
        /// Master enable override - when false, blocks all pulses
        /// Default: true (normal operation)
        parameter<bool> enable{true};

        /// Pulse width for each firing pulse in radians
        /// Range: [π/36, π/3] radians (5° to 60°)
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
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // bool parameter - use default accessor
        entry.parameters.insert("enable", _model_parameters->enable);

        // angle_wrapped parameter - convert to/from radians
        auto& pulse_width_param = _model_parameters->pulse_width;
        entry.parameters.insert("pulse_width",
            low_level_access(
                // Read: angle_wrapped -> float (radians)
                [&pulse_width_param]() -> float { return static_cast<float>(pulse_width_param.read().get_radians()); },
                // Write: float (radians) -> angle_wrapped
                [&pulse_width_param](
                    float value_rad) { pulse_width_param.write(angle_wrapped::from_radians(angle_t{value_rad})); }));

        // Enum parameter - convert to/from int
        auto& reference_type_param = _model_parameters->reference_type;
        entry.parameters.insert("reference_type",
            low_level_access(
                // Read: enum -> float (as int)
                [&reference_type_param]() -> float {
                    return static_cast<float>(static_cast<int>(reference_type_param.read()));
                },
                // Write: float (as int) -> enum
                [&reference_type_param](float value) {
                    reference_type_param.write(
                        static_cast<control::six_pulse_generator::reference_type>(static_cast<int>(value)));
                }));

        registry.insert("grid_side_pulse_generator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
