#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_speed_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Speed Controller component
 */

namespace yeet::application::sfc::integration {

class sfc_speed_controller_datamodel {
public:
    /**
     * @brief Input signals for the speed controller component
     */
    struct inputs {
        /// Enable signal for speed control mode
        signal_input<bool> enable_speed_control;

        /// Speed reference signal in per-unit (machine base)
        signal_input<speed_pu_t> speed_reference_pu;

        /// Measured machine speed in per-unit (machine base)
        signal_input<speed_pu_t> measured_speed_pu;

        signal_input<bool> freeze_integrator;
        signal_input<bool> current_controller_is_saturated;
        signal_input<bool> modulator_freeze_integrator;  // needs additional logic to handle all freezes
    };

    /**
     * @brief Output signals from the speed controller component
     */
    struct outputs {
        /// Current reference output to inner current loop
        /// Range: [-1.2, 1.2] pu (limited to protect current loop)
        signal_output<current_pu_t> current_reference;

        /// Estimated torque output (proportional to current reference)
        /// Range: [-1.2, 1.2] pu
        signal_output<torque_pu_t> torque_reference;

        /// Speed reference after rate limiting
        /// Range: [-1.2, 1.2] pu
        signal_output<speed_pu_t> speed_reference_limited;

        /// Speed control error (reference - measured)
        /// Range: [-1.0, 1.0] pu
        signal_output<speed_pu_t> speed_error;

        /// PI controller output before limiting
        /// Range: [-2.0, 2.0] pu (before saturation)
        signal_output<real_t> pi_output;

        /// Controller saturation status
        signal_output<bool> is_saturated;

        /// Speed control active status
        signal_output<bool> is_active;
    };

    /**
     * @brief Configuration parameters for the speed controller
     */
    struct parameters {
        // /// PI controller proportional gain
        // /// Range: [1.0, 100.0]
        parameter<real_t> kp = real_t(6.0);

        /// PI controller integral gain
        parameter<duration_t> ti = duration_t{ms(550.0)};

        /// Speed reference rate limit (acceleration/deceleration)
        /// Range: (0.0, 100.0] pu/s
        parameter<real_t> speed_reference_rate_limit = real_t(10.0);

        /// Speed reference magnitude limit
        parameter<speed_pu_t> speed_reference_magnitude_limit = speed_pu_t(2.0);

        /// Torque-to-current conversion factor
        parameter<real_t> torque_to_current_factor = real_t(1.0);

        /// Torque reference limits
        parameter<torque_pu_t> torque_reference_magnitude_limit = torque_pu_t{2.0};

        /// Initial speed reference
        parameter<speed_pu_t> initial_speed{speed_pu_t{0.0}};

        /// Speed-control feedforward inertia factor
        /// This factor is used to adjust the feedforward term in the speed controller.
        parameter<inertia_constant_pu_t> feedforward_inertia = inertia_constant_pu_t(0.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_speed_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        // model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // // Register outputs in signal registry
        // entry.signal_registry.insert("current_reference", _model_outputs->current_reference);
        // entry.signal_registry.insert("speed_error", _model_outputs->speed_error);
        // entry.signal_registry.insert("pi_output", _model_outputs->pi_output);
        // entry.signal_registry.insert("is_saturated", _model_outputs->is_saturated);
        // entry.signal_registry.insert("is_active", _model_outputs->is_active);
        // entry.signal_registry.insert("speed_reference_limited", _model_outputs->speed_reference_limited);
        // entry.signal_registry.insert("estimated_torque", _model_outputs->estimated_torque);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("pi_proportional_gain", _model_parameters->pi_proportional_gain);
        // entry.parameter_registry.insert("pi_integral_gain", _model_parameters->pi_integral_gain);
        // entry.parameter_registry.insert("speed_reference_rate_limit", _model_parameters->speed_reference_rate_limit);
        // entry.parameter_registry.insert("speed_reference_magnitude_limit",
        //                                 _model_parameters->speed_reference_magnitude_limit);
        // entry.parameter_registry.insert("max_current_reference", _model_parameters->max_current_reference);
        // entry.parameter_registry.insert("min_current_reference", _model_parameters->min_current_reference);
        // entry.parameter_registry.insert("anti_windup_gain", _model_parameters->anti_windup_gain);
        // entry.parameter_registry.insert("speed_error_deadband", _model_parameters->speed_error_deadband);
        // entry.parameter_registry.insert("speed_reference_filter_time_constant",
        //                                 _model_parameters->speed_reference_filter_time_constant);
        // entry.parameter_registry.insert("torque_to_current_factor", _model_parameters->torque_to_current_factor);

        // registry.insert("sfc_speed_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
