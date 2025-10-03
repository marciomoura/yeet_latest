#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
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
        signal_input<abc<voltage_pu_t>> grid_voltage_pu;
    };

    /**
     * @brief Output signals from the grid-side observer component
     */
    struct outputs {
        /// Estimated machine voltage magnitude
        yeet::signal_output<yeet::voltage_pu_t> estimated_voltage_magnitude;

        /// Estimated machine frequency (electrical)
        yeet::signal_output<yeet::frequency_t> estimated_frequency;

        /// Estimated machine frequency in per-unit
        yeet::signal_output<yeet::frequency_pu_t> estimated_frequency_pu;

        /// Estimated rotor position in radians
        yeet::signal_output<angle_wrapped> estimated_position;

        /// Filtered machine voltage in alpha-beta stationary frame
        yeet::signal_output<yeet::alphabeta<yeet::voltage_pu_t>> filtered_voltage_alphabeta;
    };

    /**
     * @brief Configuration parameters for the grid-side observer component
     */
    struct parameters {
        /// PLL proportional gain
        yeet::parameter<real_t> kp = 0.4;

        /// PLL integral time constant
        yeet::parameter<duration_t> ti = duration_t{ms(31.8)};

        /// Nominal machine frequency
        yeet::parameter<frequency_t> nominal_frequency = frequency_t{50.0};

        /// Initial machine frequency for startup
        yeet::parameter<frequency_pu_t> initial_frequency = frequency_pu_t{1.0};

        /// Frequency estimation filter cutoff frequency
        /// Range: [1.0, 50.0] Hz
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

    void register_data_model(model_registry& registry) {}

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
