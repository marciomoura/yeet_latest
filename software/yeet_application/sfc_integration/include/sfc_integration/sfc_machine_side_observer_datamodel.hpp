#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_observer_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Observer component
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_observer_datamodel {
public:
    /**
     * @brief Input signals for the machine-side observer component
     */
    struct inputs {
        /// Three-phase machine voltage measurements in per-unit (machine-side base)
        yeet::signal_input<yeet::abc<yeet::voltage_pu_t>> machine_voltage_pu;
    };

    /**
     * @brief Output signals from the machine-side observer component
     */
    struct outputs {
        /// Estimated machine voltage magnitude
        yeet::signal_output<yeet::voltage_pu_t> estimated_voltage_magnitude;

        /// Estimated machine frequency (electrical)
        yeet::signal_output<yeet::frequency_t> estimated_frequency;

        /// Estimated machine frequency in per-unit
        yeet::signal_output<yeet::frequency_pu_t> estimated_frequency_pu;

        /// Estimated machine speed in per-unit
        yeet::signal_output<yeet::speed_pu_t> estimated_speed;

        /// Estimated rotor position in radians
        yeet::signal_output<angle_wrapped> estimated_position;

        /// Filtered machine voltage in alpha-beta stationary frame
        yeet::signal_output<yeet::alphabeta<yeet::voltage_pu_t>> filtered_voltage_alphabeta;
    };

    /**
     * @brief Configuration parameters for the machine-side observer
     */
    struct parameters {
        /// PLL proportional gain
        /// Range: [0.1, 10.0]
        yeet::parameter<double> kp = 1.0;

        /// PLL integral time constant
        /// Range: [0.01, 1.0] s
        yeet::parameter<duration_t> ti = duration_t{0.1};

        /// Nominal machine frequency
        /// Range: [30.0, 100.0] Hz
        yeet::parameter<frequency_t> nominal_frequency = frequency_t{50.0};

        /// Initial machine frequency for startup
        /// Range: [0.0, 1.0] pu
        yeet::parameter<frequency_pu_t> initial_frequency = frequency_pu_t{1.0};

        /// Voltage threshold for commutation detection
        /// Range: [0.05, 0.3] pu
        yeet::parameter<double> commutation_detection_threshold = 0.15;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_observer_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry) {}

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
