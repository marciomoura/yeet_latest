#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Machine Side Controller component
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_controller_datamodel {
public:
    /**
     * @brief Input signals for the machine side controller component
     */
    struct inputs {
        /// Measured machine electrical frequency in per-unit
        signal_input<speed_pu_t> machine_speed;

        /// Measured DC link current in Amperes
        signal_input<current_pu_t> dc_link_current;
    };

    /**
     * @brief Output signals from the machine side controller component
     */
    struct outputs {
        /// Calculated firing angle
        /// Range: [π/2, π] radians
        signal_output<angle_wrapped> machine_side_firing_angle;
    };

    /**
     * @brief Configuration parameters for the machine side controller
     */
    struct parameters {
        /// Target extinction angle (gamma) for constant gamma control
        /// Range: [5.0, 30.0] degrees
        parameter<real_t> gamma_setpoint_deg = real_t(20.0);

        /// Machine-side commutation reactance at base frequency
        /// Range: [0.1, 0.5] pu
        parameter<real_t> machine_side_commutation_reactance_pu = real_t(0.059);

        /// DC current axis points for lookup table (per-unit)
        /// Default: [0.0, 0.25, 0.5, 0.75, 1.0, 1.2]
        parameter<std::array<real_t, 6>> dc_current_axis_pu = std::array<real_t, 6>{0.0, 0.25, 0.5, 0.75, 1.0, 1.2};

        /// Speed axis points for lookup table (per-unit)
        /// Default: [0.0, 0.2, 0.5, 0.75, 1.0, 1.25]
        parameter<std::array<real_t, 6>> speed_axis_pu = std::array<real_t, 6>{0.0, 0.2, 0.5, 0.75, 1.0, 1.25};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
        // entry.signal_registry.insert("current_gamma_setpoint_deg", _model_outputs->current_gamma_setpoint_deg);
        // entry.signal_registry.insert("is_active", _model_outputs->is_active);
        // entry.signal_registry.insert("is_lookup_valid", _model_outputs->is_lookup_valid);
        // entry.signal_registry.insert("firing_angle_calculated", _model_outputs->firing_angle_calculated);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("gamma_setpoint_deg", _model_parameters->gamma_setpoint_deg);
        // entry.parameter_registry.insert("machine_side_commutation_reactance_pu",
        //                                 _model_parameters->machine_side_commutation_reactance_pu);
        // entry.parameter_registry.insert("min_firing_angle", _model_parameters->min_firing_angle);
        // entry.parameter_registry.insert("max_firing_angle", _model_parameters->max_firing_angle);
        // entry.parameter_registry.insert("dc_current_axis_pu", _model_parameters->dc_current_axis_pu);
        // entry.parameter_registry.insert("speed_axis_pu", _model_parameters->speed_axis_pu);
        // entry.parameter_registry.insert("auto_recalculate_table", _model_parameters->auto_recalculate_table);
        // entry.parameter_registry.insert("firing_angle_filter_time_constant",
        //                                 _model_parameters->firing_angle_filter_time_constant);
        // entry.parameter_registry.insert("enable_external_override", _model_parameters->enable_external_override);

        // registry.insert("sfc_machine_side_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
