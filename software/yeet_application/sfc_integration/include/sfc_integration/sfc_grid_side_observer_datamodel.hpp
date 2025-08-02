#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"


/**
 * @file sfc_grid_side_observer_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Observer component
 *
 * This file defines the interface contract for the SFC Grid-Side Observer component
 * following the yeet framework datamodel pattern with proper unit types.
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_observer_datamodel {
public:
    /**
     * @brief Input signals for the grid-side observer component
     */
    struct inputs {
        /// Three-phase grid voltage measurements in per-unit (grid-side base)
        /// Range: [0.8, 1.2] pu under normal operating conditions
        yeet::signal_input<yeet::abc<yeet::voltage_pu_t>> grid_voltage_pu;

        /// Grid-side converter firing pulses for commutation detection
        /// Type: six thyristor gate signals (true = firing)
        yeet::signal_input<yeet::control::six_pulse_generator::pulses> grid_side_pulses;
    };

    /**
     * @brief Output signals from the grid-side observer component
     */
    struct outputs {
        /// Estimated grid voltage angle (electrical)
        /// Range: [0, 2π] radians
        yeet::signal_output<yeet::angle_t> estimated_angle;

        /// Estimated grid frequency (absolute)
        /// Range: [45, 65] Hz for 50 Hz systems
        yeet::signal_output<yeet::frequency_t> estimated_frequency;

        /// Estimated grid frequency in per-unit
        /// Range: [0.9, 1.3] pu
        yeet::signal_output<yeet::frequency_pu_t> estimated_frequency_pu;

        /// Estimated positive-sequence voltage magnitude
        /// Range: [0.8, 1.2] pu under normal conditions
        yeet::signal_output<yeet::voltage_pu_t> estimated_voltage_magnitude;

        /// Filtered grid voltage in alpha-beta stationary frame
        /// Provides clean voltage signals for downstream control
        yeet::signal_output<yeet::alphabeta<yeet::voltage_pu_t>> filtered_voltage_alphabeta;

        /// Commutation detection status
        /// Indicates when grid-side converter commutation is occurring
        yeet::signal_output<bool> is_commuting;

        /// PLL lock status indication
        /// Indicates whether PLL has achieved frequency lock
        yeet::signal_output<bool> is_locked;
    };

    /**
     * @brief Configuration parameters for the grid-side observer component
     */
    struct parameters {
        /// PLL proportional gain for frequency tracking
        /// Range: [50, 500] Hz
        yeet::parameter<yeet::frequency_t> pll_proportional_gain = yeet::frequency_t(200.0);

        /// PLL integral gain for steady-state accuracy
        /// Range: [10000, 100000] Hz
        yeet::parameter<yeet::frequency_t> pll_integral_gain = yeet::frequency_t(40000.0);

        /// Nominal grid frequency for initialization
        /// Range: [50, 60] Hz
        yeet::parameter<yeet::frequency_t> grid_nominal_frequency = yeet::frequency_t(50.0);

        /// Voltage threshold for commutation detection
        /// Range: [0.05, 0.3] pu
        yeet::parameter<yeet::voltage_pu_t> commutation_detection_threshold = yeet::voltage_pu_t(0.1);

        /// Time window for commutation event detection
        /// Range: [0.0005, 0.005] s
        yeet::parameter<yeet::duration_t> commutation_detection_time_window = yeet::duration_t(0.001);

        /// Frequency error threshold for PLL lock detection
        /// Range: [0.01, 0.1] pu
        yeet::parameter<yeet::voltage_pu_t> lock_detection_threshold = yeet::voltage_pu_t(0.05);

        /// Time window for PLL lock verification
        /// Range: [0.05, 0.5] s
        yeet::parameter<yeet::duration_t> lock_detection_time_window = yeet::duration_t(0.1);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_observer_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry) {
        yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("estimated_angle", _model_outputs->estimated_angle);
        entry.signal_registry.insert("estimated_frequency", _model_outputs->estimated_frequency);
        entry.signal_registry.insert("estimated_frequency_pu", _model_outputs->estimated_frequency_pu);
        entry.signal_registry.insert("estimated_voltage_magnitude", _model_outputs->estimated_voltage_magnitude);
        entry.signal_registry.insert("filtered_voltage_alphabeta", _model_outputs->filtered_voltage_alphabeta);
        entry.signal_registry.insert("is_commuting", _model_outputs->is_commuting);
        entry.signal_registry.insert("is_locked", _model_outputs->is_locked);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("pll_proportional_gain", _model_parameters->pll_proportional_gain);
        entry.parameter_registry.insert("pll_integral_gain", _model_parameters->pll_integral_gain);
        entry.parameter_registry.insert("grid_nominal_frequency", _model_parameters->grid_nominal_frequency);
        entry.parameter_registry.insert("commutation_detection_threshold",
                                        _model_parameters->commutation_detection_threshold);
        entry.parameter_registry.insert("commutation_detection_time_window",
                                        _model_parameters->commutation_detection_time_window);
        entry.parameter_registry.insert("lock_detection_threshold", _model_parameters->lock_detection_threshold);
        entry.parameter_registry.insert("lock_detection_time_window", _model_parameters->lock_detection_time_window);

        registry.insert("sfc_grid_side_observer", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
