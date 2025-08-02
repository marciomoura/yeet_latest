#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
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
        /// Speed reference signal in per-unit (machine base)
        /// Range: [-1.2, 1.2] pu for motoring/generating operation
        signal_input<speed_pu_t> speed_reference_pu;

        /// Measured machine speed in per-unit (machine base)
        /// Range: [-1.5, 1.5] pu (includes measurement range)
        signal_input<speed_pu_t> measured_speed_pu;

        /// Measured DC link current for anti-windup logic
        /// Range: [-1.5, 1.5] pu
        signal_input<current_pu_t> measured_current_pu;

        /// Enable signal for speed control mode
        signal_input<bool> enable_speed_control;

        /// External current reference for torque mode operation
        /// Range: [-1.2, 1.2] pu
        signal_input<current_pu_t> external_current_reference_pu;
    };

    /**
     * @brief Output signals from the speed controller component
     */
    struct outputs {
        /// Current reference output to inner current loop
        /// Range: [-1.2, 1.2] pu (limited to protect current loop)
        signal_output<current_pu_t> current_reference;

        /// Speed control error (reference - measured)
        /// Range: [-1.0, 1.0] pu
        signal_output<speed_pu_t> speed_error;

        /// PI controller output before limiting
        /// Range: [-2.0, 2.0] pu (before saturation)
        signal_output<current_pu_t> pi_output;

        /// Controller saturation status
        signal_output<bool> is_saturated;

        /// Speed control active status
        signal_output<bool> is_speed_control_active;

        /// Speed reference after rate limiting
        /// Range: [-1.2, 1.2] pu
        signal_output<speed_pu_t> speed_reference_limited;

        /// Estimated torque output (proportional to current reference)
        /// Range: [-1.2, 1.2] pu
        signal_output<torque_pu_t> estimated_torque;
    };

    /**
     * @brief Configuration parameters for the speed controller
     */
    struct parameters {
        /// PI controller proportional gain
        /// Range: [1.0, 100.0]
        parameter<real_t> pi_proportional_gain = real_t(10.0);

        /// PI controller integral gain
        /// Range: [10.0, 1000.0]
        parameter<real_t> pi_integral_gain = real_t(100.0);

        /// Speed reference rate limit (acceleration/deceleration)
        /// Range: [0.1, 10.0] pu/s
        parameter<real_t> speed_reference_rate_limit = real_t(2.0);

        /// Speed reference magnitude limit
        /// Range: [1.0, 1.2] pu
        parameter<speed_pu_t> speed_reference_magnitude_limit = speed_pu_t(1.1);

        /// Current reference output limit (positive)
        /// Range: [1.0, 1.5] pu
        parameter<current_pu_t> max_current_reference = current_pu_t(1.1);

        /// Current reference output limit (negative)
        /// Range: [-1.5, -1.0] pu
        parameter<current_pu_t> min_current_reference = current_pu_t(-1.1);

        /// Anti-windup gain for integral term protection
        /// Range: [0.05, 0.5]
        parameter<real_t> anti_windup_gain = real_t(0.1);

        /// Deadband for speed error (reduces noise sensitivity)
        /// Range: [0.0, 0.01] pu
        parameter<speed_pu_t> speed_error_deadband = speed_pu_t(0.001);

        /// Low-pass filter time constant for speed reference
        /// Range: [0.001, 0.1] s
        parameter<duration_t> speed_reference_filter_time_constant = duration_t(0.01);

        /// Torque-to-current conversion factor
        /// Range: [0.5, 1.5]
        parameter<real_t> torque_to_current_factor = real_t(1.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_speed_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {
        // model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // // Register outputs in signal registry
        // entry.signal_registry.insert("current_reference", _model_outputs->current_reference);
        // entry.signal_registry.insert("speed_error", _model_outputs->speed_error);
        // entry.signal_registry.insert("pi_output", _model_outputs->pi_output);
        // entry.signal_registry.insert("is_saturated", _model_outputs->is_saturated);
        // entry.signal_registry.insert("is_speed_control_active", _model_outputs->is_speed_control_active);
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
