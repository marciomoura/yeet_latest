#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_dc_link_measurements_datamodel.hpp
 * @brief Datamodel specification for SFC DC Link Measurements component
 */

namespace yeet::application::sfc::integration {

class sfc_dc_link_measurements_datamodel {
public:
    /**
     * @brief Input signals for the DC link measurements component
     */
    struct inputs {
        // signal_input<voltage_t> dc_voltage;
        signal_input<current_t> dc_current;
    };

    /**
     * @brief Output signals from the DC link measurements component
     */
    struct outputs {
        // signal_output<voltage_pu_t> dc_voltage_pu;
        // signal_output<voltage_pu_t> dc_voltage_filtered_pu;
        signal_output<current_pu_t> dc_current_pu;
        signal_output<current_pu_t> dc_current_filtered_pu;
        signal_output<current_pu_t> dc_current_heavy_filtered_pu;
    };

    /**
     * @brief Configuration parameters for the DC link measurements
     */
    struct parameters {
        parameter<frequency_t> filter_cutoff_frequency = frequency_t(100.0);
        parameter<frequency_t> heavy_filter_cutoff_frequency = frequency_t(10.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_dc_link_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
        // entry.signal_registry.insert("dc_voltage_pu", _model_outputs->dc_voltage_pu);
        // entry.signal_registry.insert("dc_current_pu", _model_outputs->dc_current_pu);
        // entry.signal_registry.insert("dc_voltage_filtered_pu", _model_outputs->dc_voltage_filtered_pu);
        // entry.signal_registry.insert("dc_current_filtered_pu", _model_outputs->dc_current_filtered_pu);
        // entry.signal_registry.insert("dc_power_pu", _model_outputs->dc_power_pu);
        // entry.signal_registry.insert("is_valid", _model_outputs->is_valid);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("voltage_scale_factor", _model_parameters->voltage_scale_factor);
        // entry.parameter_registry.insert("current_scale_factor", _model_parameters->current_scale_factor);
        // entry.parameter_registry.insert("filter_cutoff_frequency", _model_parameters->filter_cutoff_frequency);

        // registry.insert("sfc_dc_link_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
