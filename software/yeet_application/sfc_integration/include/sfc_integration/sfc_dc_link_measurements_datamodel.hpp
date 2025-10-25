#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
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
        signal_input<abc<current_pu_t>> grid_side_current;
    };

    /**
     * @brief Output signals from the DC link measurements component
     */
    struct outputs {
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
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register parameters - frequency_t type needs conversion to float (Hz)
        auto& filter_cutoff_param = _model_parameters->filter_cutoff_frequency;
        entry.parameters.insert("filter_cutoff_frequency",
            low_level_access(
                // Read: frequency_t -> float (Hz)
                [&filter_cutoff_param]() -> float { return static_cast<float>(filter_cutoff_param.read()); },
                // Write: float (Hz) -> frequency_t
                [&filter_cutoff_param](float value_hz) { filter_cutoff_param.write(frequency_t{value_hz}); }));

        auto& heavy_filter_cutoff_param = _model_parameters->heavy_filter_cutoff_frequency;
        entry.parameters.insert(
            "heavy_filter_cutoff_frequency", low_level_access(
                                                 // Read: frequency_t -> float (Hz)
                                                 [&heavy_filter_cutoff_param]() -> float {
                                                     return static_cast<float>(heavy_filter_cutoff_param.read());
                                                 },
                                                 // Write: float (Hz) -> frequency_t
                                                 [&heavy_filter_cutoff_param](float value_hz) {
                                                     heavy_filter_cutoff_param.write(frequency_t{value_hz});
                                                 }));

        registry.insert("dc_link_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
