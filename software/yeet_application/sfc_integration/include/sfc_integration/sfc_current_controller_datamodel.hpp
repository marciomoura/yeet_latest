#pragma once

#include <chrono>

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
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

        /// @brief  Indicates if the current reference exceeded the magnitude limit
        signal_output<bool> is_reference_current_limited;

        /// @brief Indicates if the measured current exceeded the magnitude limit
        signal_output<bool> is_current_measured_above_magnitude_limit;
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

        /// Current reference for rectifier operation
        parameter<current_pu_t> current_reference_rectifier_operation = current_pu_t(0.0);
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

        entry.parameters.insert("kp", _model_parameters->kp);
        entry.parameters.insert("ti", yeet::utils::make_units_accessor(_model_parameters->ti));
        entry.parameters.insert(
            "min_firing_angle", yeet::utils::make_units_accessor(_model_parameters->min_firing_angle));
        entry.parameters.insert(
            "max_firing_angle", yeet::utils::make_units_accessor(_model_parameters->max_firing_angle));
        entry.parameters.insert("current_reference_magnitude_limit",
            yeet::utils::make_units_accessor(_model_parameters->current_reference_magnitude_limit));
        entry.parameters.insert(
            "current_reference_rectifier_operation", _model_parameters->current_reference_rectifier_operation);

        registry.insert("current_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
