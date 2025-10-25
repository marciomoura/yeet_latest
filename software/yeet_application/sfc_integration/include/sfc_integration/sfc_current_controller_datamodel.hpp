#pragma once

#include <chrono>

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_current_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Current Controller component
 */

namespace yeet::application::sfc::integration {

class sfc_current_controller_datamodel {
public:
    /**
     * @brief Input signals for the current controller component
     */
    struct inputs {
        signal_input<current_pu_t> dc_link_current_reference;

        signal_input<current_pu_t> dc_link_current_measured;

        signal_input<voltage_pu_t> grid_side_voltage_magnitude;

        signal_input<voltage_pu_t> machine_side_voltage_magnitude;

        signal_input<bool> freeze_integrator;
        signal_input<bool> modulator_freeze_integrator;  // needs additional logic to handle all freezes
    };

    /**
     * @brief Output signals from the current controller component
     */
    struct outputs {
        /// Grid-side rectifier firing angle reference
        signal_output<angle_wrapped> firing_angle;

        /// Current control error (reference - measured)
        signal_output<current_pu_t> current_error;

        /// PI controller output before limiting
        signal_output<angle_wrapped> pi_output;

        /// Controller saturation status
        signal_output<bool> current_controller_is_saturated;

        /// Control loop active status
        signal_output<bool> is_active;
    };

    /**
     * @brief Configuration parameters for the current controller
     */
    struct parameters {
        /// PI controller proportional gain
        parameter<real_t> kp{0.7439 / 1.35};

        /// PI controller integral time [seconds]
        parameter<duration_t> ti{ms(101.64)};

        /// Minimum firing angle limit [radians]
        parameter<angle_wrapped> min_firing_angle = angle_wrapped(0.0);

        /// Maximum firing angle limit [radians]
        parameter<angle_wrapped> max_firing_angle = angle_wrapped(1.5708);  // π/2

        /// Current reference magnitude limit [pu]
        /// Applied symmetrically: [-limit, +limit]
        parameter<current_pu_t> current_reference_magnitude_limit = current_pu_t(2.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_current_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Simple float/real_t parameters - use default accessor
        entry.parameters.insert("kp", _model_parameters->kp);

        // duration_t parameter - extract value in seconds (SI unit)
        // Capture parameter by reference to avoid dangling pointer from temporary datamodel
        auto& ti_param = _model_parameters->ti;
        entry.parameters.insert(
            "ti", low_level_access(
                      // Read: duration_t -> float (seconds)
                      [&ti_param]() -> float { return static_cast<float>(ti_param.read().value()); },
                      // Write: float (seconds) -> duration_t
                      [&ti_param](float value_seconds) { ti_param.write(duration_t{value_seconds}); }));

        // angle_wrapped parameters - convert to/from radians
        // Capture parameters by reference to avoid dangling pointer from temporary datamodel
        auto& max_firing_angle_param = _model_parameters->max_firing_angle;
        entry.parameters.insert(
            "max_firing_angle", low_level_access(
                                    // Read: angle_wrapped -> float (radians)
                                    [&max_firing_angle_param]() -> float {
                                        return static_cast<float>(max_firing_angle_param.read().get_radians());
                                    },
                                    // Write: float (radians) -> angle_wrapped
                                    [&max_firing_angle_param](float value_rad) {
                                        max_firing_angle_param.write(angle_wrapped::from_radians(angle_t{value_rad}));
                                    }));

        auto& min_firing_angle_param = _model_parameters->min_firing_angle;
        entry.parameters.insert(
            "min_firing_angle", low_level_access(
                                    // Read: angle_wrapped -> float (radians)
                                    [&min_firing_angle_param]() -> float {
                                        return static_cast<float>(min_firing_angle_param.read().get_radians());
                                    },
                                    // Write: float (radians) -> angle_wrapped
                                    [&min_firing_angle_param](float value_rad) {
                                        min_firing_angle_param.write(angle_wrapped::from_radians(angle_t{value_rad}));
                                    }));

        // current_pu_t parameter - extract/construct unit type
        // Capture parameter by reference to avoid dangling pointer from temporary datamodel
        auto& current_limit_param = _model_parameters->current_reference_magnitude_limit;
        entry.parameters.insert("current_reference_magnitude_limit",
            low_level_access(
                // Read: current_pu_t -> float (per-unit value)
                [&current_limit_param]() -> float { return static_cast<float>(current_limit_param.read()); },
                // Write: float (per-unit value) -> current_pu_t
                [&current_limit_param](float value_pu) { current_limit_param.write(current_pu_t{value_pu}); }));

        registry.insert("current_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
