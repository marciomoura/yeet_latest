#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_simulator_control_datamodel.hpp
 * @brief Datamodel specification for host simulator runtime control
 *
 * Provides runtime control over simulator execution parameters such as
 * execution frequency. Only available in host builds.
 */

namespace yeet::application::sfc::integration {

class sfc_simulator_control_datamodel {
public:
    /**
     * @brief Input signals (none for simulator control)
     */
    struct inputs {
        // No runtime inputs for simulator control
    };

    /**
     * @brief Output signals (none for simulator control)
     */
    struct outputs {
        // No runtime outputs for simulator control
    };

    /**
     * @brief Configuration parameters for simulator control
     */
    struct parameters {
        /// Throttle factor for simulation speed control
        /// - throttle = 0.0: Run as fast as possible (no real-time constraint)
        /// - throttle = 1.0: Run at 1:1 real-time (simulation time = wall-clock time)
        /// - throttle > 1.0: Run slower than real-time (e.g., 2.0 = half speed for debugging)
        /// - throttle < 1.0: Run faster than real-time (e.g., 0.5 = 2x speed)
        /// Default: 0.0 (maximum speed)
        parameter<real_t> throttle_factor = real_t{0.0f};

        // Allow default copy/move
        parameters() = default;
        parameters(const parameters&) = default;
        parameters(parameters&&) = default;
        parameters& operator=(const parameters&) = default;
        parameters& operator=(parameters&&) = default;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_simulator_control_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert(
            "throttle_factor", yeet::utils::make_units_accessor(_model_parameters->throttle_factor));

        registry.insert("simulator_control", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
