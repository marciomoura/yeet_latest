#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
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

        /// Unified freeze command from supervisor
        signal_input<bool> freeze_integrator;

        /// Preset request from supervisor
        signal_input<bool> preset_request;

        /// Saturation status feedback
        signal_input<bool> is_saturated;
    };

    /**
     * @brief Output signals from the speed controller component
     */
    struct outputs {
        /// Torque reference output to torque controller
        /// Range: [-1.2, 1.2] pu (limited to protect downstream control)
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
        /// PI controller proportional gain
        /// Range: [1.0, 100.0]
        parameter<real_t> kp = real_t(6.32);

        /// PI controller integral time constant
        /// Range: (0.0, 10.0] seconds
        parameter<duration_t> ti = duration_t{ms(2200.0)};

        /// Torque reference magnitude limit [pu]
        /// Applied symmetrically: [-limit, +limit]
        parameter<torque_pu_t> torque_reference_magnitude_limit = torque_pu_t{2.0};

        /// Initial speed for controller reset [pu]
        parameter<speed_pu_t> initial_speed{speed_pu_t{0.0}};

        /// Feedforward inertia constant [pu]
        /// Set to 0.0 to disable feedforward
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
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert("kp", _model_parameters->kp);
        entry.parameters.insert("ti", yeet::utils::make_units_accessor(_model_parameters->ti));
        entry.parameters.insert("torque_reference_magnitude_limit",
            yeet::utils::make_units_accessor(_model_parameters->torque_reference_magnitude_limit));
        entry.parameters.insert("initial_speed", yeet::utils::make_units_accessor(_model_parameters->initial_speed));
        entry.parameters.insert(
            "feedforward_inertia", yeet::utils::make_units_accessor(_model_parameters->feedforward_inertia));

        registry.insert("speed_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
