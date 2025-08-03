#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"

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
        /// Current reference signal in per-unit (DC link base)
        /// Range: [-1.2, 1.2] pu
        signal_input<current_pu_t> current_reference_pu;

        /// Measured DC link current in per-unit
        /// Range: [-1.5, 1.5] pu
        signal_input<current_pu_t> dc_link_current_measured_pu;

        /// Ideal no-load DC voltage from grid-side rectifier
        /// Range: [0.8, 1.2] pu
        signal_input<voltage_pu_t> ideal_dc_voltage_pu;

        /// Estimated inverter back-EMF
        /// Range: [0.0, 1.2] pu
        signal_input<voltage_pu_t> inverter_back_emf_machine_pu;
    };

    /**
     * @brief Output signals from the current controller component
     */
    struct outputs {
        /// Grid-side rectifier firing angle reference
        /// Range: [0.0, π/2] radians
        signal_output<angle_wrapped> firing_angle_reference;

        /// Current control error (reference - measured)
        /// Range: [-1.0, 1.0] pu
        signal_output<current_pu_t> current_error;

        /// PI controller output before limiting
        signal_output<angle_wrapped> pi_output;

        /// Controller saturation status
        signal_output<bool> is_saturated;

        /// Control loop active status
        signal_output<bool> is_active;
    };

    /**
     * @brief Configuration parameters for the current controller
     */
    struct parameters {
        /// PI controller proportional gain
        /// Range: [1.0, 20.0]
        parameter<real_t> pi_proportional_gain = real_t(5.0);

        /// PI controller integral gain
        /// Range: [100.0, 2000.0]
        parameter<real_t> pi_integral_gain = real_t(500.0);

        /// Minimum firing angle limit
        /// Range: [0.0, π/6] radians
        parameter<angle_wrapped> min_firing_angle = angle_wrapped(0.0);

        /// Maximum firing angle limit
        /// Range: [π/4, π/2] radians
        parameter<angle_wrapped> max_firing_angle = angle_wrapped(1.5708);  // π/2

        /// Current reference rate limit
        /// Range: [1.0, 50.0] pu/s
        parameter<real_t> current_reference_rate_limit = real_t(10.0);

        /// Current reference magnitude limit
        /// Range: [1.0, 1.5] pu
        parameter<current_pu_t> current_reference_magnitude_limit = current_pu_t(1.1);

        /// Feedforward gain for ideal DC voltage compensation
        /// Range: [0.0, 1.0]
        parameter<real_t> feedforward_dc_voltage_gain = real_t(0.8);

        /// Feedforward gain for back-EMF compensation
        /// Range: [0.0, 1.0]
        parameter<real_t> feedforward_back_emf_gain = real_t(0.5);

        /// Anti-windup gain for integral term protection
        /// Range: [0.05, 0.5]
        parameter<real_t> anti_windup_gain = real_t(0.1);
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
        // model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // // Register outputs in signal registry
        // entry.signal_registry.insert("firing_angle_reference", _model_outputs->firing_angle_reference);
        // entry.signal_registry.insert("current_error", _model_outputs->current_error);
        // entry.signal_registry.insert("pi_output", _model_outputs->pi_output);
        // entry.signal_registry.insert("is_saturated", _model_outputs->is_saturated);
        // entry.signal_registry.insert("is_active", _model_outputs->is_active);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("pi_proportional_gain", _model_parameters->pi_proportional_gain);
        // entry.parameter_registry.insert("pi_integral_gain", _model_parameters->pi_integral_gain);
        // entry.parameter_registry.insert("min_firing_angle", _model_parameters->min_firing_angle);
        // entry.parameter_registry.insert("max_firing_angle", _model_parameters->max_firing_angle);
        // entry.parameter_registry.insert(
        //     "current_reference_rate_limit", _model_parameters->current_reference_rate_limit);
        // entry.parameter_registry.insert(
        //     "current_reference_magnitude_limit", _model_parameters->current_reference_magnitude_limit);
        // entry.parameter_registry.insert("feedforward_dc_voltage_gain",
        // _model_parameters->feedforward_dc_voltage_gain); entry.parameter_registry.insert("feedforward_back_emf_gain",
        // _model_parameters->feedforward_back_emf_gain); entry.parameter_registry.insert("anti_windup_gain",
        // _model_parameters->anti_windup_gain);

        // registry.insert("sfc_current_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
