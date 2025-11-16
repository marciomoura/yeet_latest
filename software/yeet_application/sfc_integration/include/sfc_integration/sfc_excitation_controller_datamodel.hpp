#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_excitation_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Excitation Controller component
 */

namespace yeet::application::sfc::integration {

class sfc_excitation_controller_datamodel {
public:
    /**
     * @brief Input signals for the excitation controller component
     */
    struct inputs {
        /// Machine rotor speed [pu]
        signal_input<speed_pu_t> machine_rotor_speed_pu;

        /// Release excitation command (enables field current output) [bool]
        signal_input<bool> release_excitation;
    };

    /**
     * @brief Output signals from the excitation controller component
     */
    struct outputs {
        /// Field current reference [pu]
        signal_output<current_pu_t> field_current_reference;

        /// Pre-fluxing elapsed time [s]
        signal_output<duration_t> prefluxing_elapsed_time;

        /// Field established flag - true when flux buildup timer has elapsed [bool]
        signal_output<bool> is_field_established;

        /// Excitation start command - true when excitation should be energized [bool]
        signal_output<bool> excitation_start_command;

        /// Excitation stop command - true when excitation should be de-energized [bool]
        signal_output<bool> excitation_stop_command;
    };

    /**
     * @brief Configuration parameters for the excitation controller
     */
    struct parameters {
        /// Constant field current setpoint [pu]
        parameter<current_pu_t> field_current_setpoint = current_pu_t{1.0};

        /// Speed threshold for constant flux region [pu]
        /// Below this speed, field current remains constant regardless of speed
        parameter<speed_pu_t> constant_flux_speed_threshold = speed_pu_t{1.0};

        /// Pre-fluxing duration [s]
        /// Time required to establish the magnetic field before releasing excitation
        parameter<duration_t> prefluxing_duration = duration_t{2.0};

        /// Enable override for field current reference output
        /// When true, field_current_reference_override_value is used instead of controller output
        parameter<bool> override_field_current_reference = false;

        /// Override value for field current reference [pu]
        /// Used when override_field_current_reference is true
        parameter<current_pu_t> field_current_reference_override_value = current_pu_t{0.0};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_excitation_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
            "field_current_setpoint", yeet::utils::make_units_accessor(_model_parameters->field_current_setpoint));
        entry.parameters.insert("constant_flux_speed_threshold",
            yeet::utils::make_units_accessor(_model_parameters->constant_flux_speed_threshold));
        entry.parameters.insert(
            "prefluxing_duration", yeet::utils::make_units_accessor(_model_parameters->prefluxing_duration));
        entry.parameters.insert("override_field_current_reference",
            yeet::utils::make_bool_accessor(_model_parameters->override_field_current_reference));
        entry.parameters.insert("field_current_reference_override_value",
            yeet::utils::make_units_accessor(_model_parameters->field_current_reference_override_value));

        registry.insert("excitation_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
