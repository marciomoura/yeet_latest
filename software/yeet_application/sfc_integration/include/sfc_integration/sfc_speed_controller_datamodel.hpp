#pragma once

#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
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
        /// Enable signal for speed control mode (pulse release from supervisor)
        signal_input<bool> release_pulses;

        /// Speed reference signal in per-unit (machine base)
        signal_input<speed_pu_t> speed_reference;

        /// Measured machine speed in per-unit (machine base)
        signal_input<speed_pu_t> measured_speed;

        /// Unified freeze command from supervisor
        signal_input<bool> freeze_integrator;

        /// Preset request from supervisor
        signal_input<bool> preset_request;

        /// Saturation status feedback
        signal_input<bool> is_saturated;

        /// Applied torque reference from torque controller for bumpless transfer initialization
        signal_input<torque_pu_t> applied_torque_reference;
    };

    /**
     * @brief Output signals from the speed controller component
     */
    struct outputs {
        /// Torque reference output to torque controller [pu]
        signal_output<torque_pu_t> torque_reference;

        /// PI controller output before limiting [pu]
        signal_output<real_t> pi_output;

        /// Controller saturation status
        signal_output<bool> is_saturated;
    };

    /**
     * @brief Configuration parameters for the speed controller
     */
    struct parameters {
        /// PI controller proportional gain
        parameter<real_t> kp = real_t(6.32);

        /// PI controller integral time constant [s]
        parameter<duration_t> ti = duration_t{ms(2200.0)};

        /// Torque reference magnitude limit [pu]
        parameter<torque_pu_t> torque_reference_magnitude_limit = torque_pu_t{1.2};

        /// Initial speed for controller reset [pu]
        parameter<speed_pu_t> initial_speed{speed_pu_t{0.0}};

        /// Feedforward inertia constant [pu]
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "speed_controller";
        handler.register_parameter(component_name, "kp", yeet::utils::make_units_accessor(_model_parameters->kp));
        handler.register_parameter(component_name, "ti", yeet::utils::make_units_accessor(_model_parameters->ti));
        handler.register_parameter(component_name, "torque_reference_magnitude_limit",
            yeet::utils::make_units_accessor(_model_parameters->torque_reference_magnitude_limit));
        handler.register_parameter(
            component_name, "initial_speed", yeet::utils::make_units_accessor(_model_parameters->initial_speed));
        handler.register_parameter(component_name, "feedforward_inertia",
            yeet::utils::make_units_accessor(_model_parameters->feedforward_inertia));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
