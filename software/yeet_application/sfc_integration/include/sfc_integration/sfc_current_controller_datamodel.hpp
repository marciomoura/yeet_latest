#pragma once

#include <chrono>

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
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
        /// DC link current reference input [pu]
        signal_input<current_pu_t> current_reference;

        /// Measured DC link current input [pu]
        signal_input<current_pu_t> current_measured;

        /// Grid-side voltage magnitude input [pu]
        signal_input<voltage_pu_t> grid_voltage_magnitude;

        /// Machine-side voltage magnitude input [pu]
        signal_input<voltage_pu_t> machine_voltage_magnitude;

        /// Unified freeze command from supervisor (ORed from all freeze sources)
        signal_input<bool> freeze_integrator;

        /// Preset request from supervisor (triggers preset sequence)
        signal_input<bool> preset_request;
    };

    /**
     * @brief Output signals from the current controller component
     */
    struct outputs {
        /// Grid-side rectifier firing angle reference [rad]
        signal_output<angle_wrapped> firing_angle;

        /// Current control error (reference - measured) [pu]
        signal_output<current_pu_t> current_error;

        /// Full current reference output [pu]
        signal_output<current_pu_t> current_reference;

        /// PI controller output before limiting [pu]
        signal_output<real_t> pi_output;

        /// Controller saturation status
        signal_output<bool> is_saturated;

        /// Control loop active status
        signal_output<bool> is_active;

        /// Indicates if the current reference exceeded the magnitude limit
        signal_output<bool> is_reference_current_limited;

        /// Indicates if the measured current exceeded the magnitude limit
        signal_output<bool> is_current_measured_above_magnitude_limit;

        /// Preset sequence complete acknowledgment
        signal_output<bool> preset_complete;
    };

    /**
     * @brief Configuration parameters for the current controller
     */
    struct parameters {
        /// PI controller proportional gain
        parameter<real_t> kp{0.7439 / 1.35};

        /// PI controller integral time constant [s]
        // parameter<duration_t> ti{ms(101.64)};
        parameter<duration_t> ti{ms(201.64)};

        /// Minimum firing angle limit [rad]
        parameter<angle_wrapped> min_firing_angle = angle_wrapped(0.0);

        /// Maximum firing angle limit [rad]
        parameter<angle_wrapped> max_firing_angle = angle_wrapped::from_degrees(90.0);

        /// Current reference magnitude limit [pu]
        parameter<current_pu_t> current_reference_magnitude_limit = current_pu_t(2.0);

        /// Minimum PI controller output limit [pu voltage]
        parameter<real_t> pi_output_min = real_t(-1.0);

        /// Maximum PI controller output limit [pu voltage]
        parameter<real_t> pi_output_max = real_t(1.0);

        /// Enable current reference override (force current reference to parameter) [bool]
        parameter<bool> override_current_reference{false};

        /// Forced DC link current reference when override is enabled [pu]
        parameter<current_pu_t> override_current_reference_value{current_pu_t(0.0)};
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "current_controller";
        handler.register_parameter(component_name, "kp", yeet::utils::make_units_accessor(_model_parameters->kp));
        handler.register_parameter(component_name, "ti", yeet::utils::make_units_accessor(_model_parameters->ti));
        handler.register_parameter(
            component_name, "min_firing_angle", yeet::utils::make_units_accessor(_model_parameters->min_firing_angle));
        handler.register_parameter(
            component_name, "max_firing_angle", yeet::utils::make_units_accessor(_model_parameters->max_firing_angle));
        handler.register_parameter(component_name, "current_reference_magnitude_limit",
            yeet::utils::make_units_accessor(_model_parameters->current_reference_magnitude_limit));
        handler.register_parameter(
            component_name, "pi_output_min", yeet::utils::make_units_accessor(_model_parameters->pi_output_min));
        handler.register_parameter(
            component_name, "pi_output_max", yeet::utils::make_units_accessor(_model_parameters->pi_output_max));
        handler.register_parameter(component_name, "override_current_reference",
            yeet::utils::make_bool_accessor(_model_parameters->override_current_reference));
        handler.register_parameter(component_name, "override_current_reference_value",
            yeet::utils::make_units_accessor(_model_parameters->override_current_reference_value));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
