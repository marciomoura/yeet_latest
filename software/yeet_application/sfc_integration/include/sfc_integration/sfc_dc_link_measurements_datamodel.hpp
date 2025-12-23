#pragma once

#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
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
        /// Grid current in abc frame [pu]
        signal_input<abc<current_pu_t>> grid_current_abc;
    };

    /**
     * @brief Output signals from the DC link measurements component
     */
    struct outputs {
        /// DC link current (raw) [pu]
        signal_output<current_pu_t> dc_current_pu;

        /// DC link current (filtered) [pu]
        signal_output<current_pu_t> dc_current_filtered_pu;

        /// DC link current (heavily filtered) [pu]
        signal_output<current_pu_t> dc_current_heavy_filtered_pu;
    };

    /**
     * @brief Configuration parameters for the DC link measurements
     */
    struct parameters {
        /// Filter cutoff frequency [Hz]
        parameter<frequency_t> filter_cutoff_frequency = frequency_t(100.0);

        /// Heavy filter cutoff frequency [Hz]
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "dc_link_measurements";
        handler.register_parameter(component_name, "filter_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->filter_cutoff_frequency));
        handler.register_parameter(component_name, "heavy_filter_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->heavy_filter_cutoff_frequency));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
