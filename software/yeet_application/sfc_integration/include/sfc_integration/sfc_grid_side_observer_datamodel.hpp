#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

class sfc_grid_side_observer_datamodel {
public:
    /**
     * @brief Input signals for the grid-side observer component
     */
    struct inputs {
        /// Three-phase grid voltage measurements in per-unit (grid-side base)
        signal_input<abc<voltage_pu_t>> grid_voltage_abc;
    };

    /**
     * @brief Output signals from the grid-side observer component
     */
    struct outputs {
        /// Estimated voltage magnitude [pu]
        yeet::signal_output<yeet::voltage_pu_t> estimated_voltage_magnitude;

        /// Estimated frequency [Hz]
        yeet::signal_output<yeet::frequency_t> estimated_frequency;

        /// Estimated frequency in per-unit [pu]
        yeet::signal_output<yeet::frequency_pu_t> estimated_frequency_pu;

        /// Estimated voltage phase angle [rad]
        yeet::signal_output<angle_wrapped> estimated_reference_angle;

        /// Filtered voltage in alpha-beta stationary frame [pu]
        yeet::signal_output<yeet::alphabeta<yeet::voltage_pu_t>> estimated_voltage_alphabeta_filtered;
    };

    /**
     * @brief Configuration parameters for the grid-side observer component
     */
    struct parameters {
        /// PLL proportional gain
        yeet::parameter<real_t> kp = 0.4;

        /// PLL integral time constant [s]
        yeet::parameter<duration_t> ti = duration_t{ms(31.8)};

        /// Nominal grid frequency [Hz]
        yeet::parameter<frequency_t> nominal_frequency = frequency_t{50.0};

        /// Initial frequency estimate for startup [pu]
        yeet::parameter<frequency_pu_t> initial_frequency = frequency_pu_t{1.0};

        /// Frequency estimation filter cutoff frequency [Hz]
        yeet::parameter<frequency_t> frequency_filter_cutoff = frequency_t{10.0};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_observer_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert("kp", yeet::utils::make_units_accessor(_model_parameters->kp));
        entry.parameters.insert("ti", yeet::utils::make_units_accessor(_model_parameters->ti));
        entry.parameters.insert(
            "nominal_frequency", yeet::utils::make_units_accessor(_model_parameters->nominal_frequency));
        entry.parameters.insert(
            "initial_frequency", yeet::utils::make_units_accessor(_model_parameters->initial_frequency));
        entry.parameters.insert(
            "frequency_filter_cutoff", yeet::utils::make_units_accessor(_model_parameters->frequency_filter_cutoff));

        registry.insert("observer", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
