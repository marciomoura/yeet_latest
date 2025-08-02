#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"


/**
 * @file sfc_grid_side_modulator_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Modulator component
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_modulator_datamodel {
public:
    /**
     * @brief Input signals for the grid-side modulator component
     */
    struct inputs {
        /// Firing angle reference from current controller
        /// Range: [0.0, π/2] radians (0° to 90°)
        signal_input<angle_t> firing_angle_reference;

        /// Grid voltage angle from grid-side observer
        /// Range: [0, 2π] radians
        signal_input<angle_t> grid_voltage_angle;

        /// Enable command for pulse generation
        signal_input<bool> enable_command;

        /// Grid frequency for pulse timing adjustment
        /// Range: [45, 65] Hz
        signal_input<frequency_t> grid_frequency;
    };

    /**
     * @brief Output signals from the grid-side modulator component
     */
    struct outputs {
        /// Six-pulse firing signals for grid-side thyristors
        signal_output<control::six_pulse_generator::pulses> firing_pulses;

        /// Actual firing angle being used
        /// Range: [0.0, π/2] radians
        signal_output<angle_t> actual_firing_angle;

        /// Modulator enabled status
        signal_output<bool> is_enabled;

        /// Pulse generator synchronization status
        signal_output<bool> is_synchronized;
    };

    /**
     * @brief Configuration parameters for the grid-side modulator
     */
    struct parameters {
        /// Minimum firing angle limit
        /// Range: [0.0, π/6] radians
        parameter<angle_t> min_firing_angle = angle_t(0.0);

        /// Maximum firing angle limit
        /// Range: [π/4, π/2] radians
        parameter<angle_t> max_firing_angle = angle_t(1.5708);  // π/2

        /// Pulse width for thyristor firing
        /// Range: [π/36, π/6] radians (5° to 30°)
        parameter<angle_t> pulse_width = angle_t(0.5236);  // π/6 = 30°

        /// Deadtime between complementary pulses
        /// Range: [0.0001, 0.001] s
        parameter<duration_t> deadtime = duration_t(0.0002);

        /// Nominal grid frequency for initialization
        /// Range: [50, 60] Hz
        parameter<frequency_t> nominal_grid_frequency = frequency_t(50.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_modulator_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("firing_pulses", _model_outputs->firing_pulses);
        entry.signal_registry.insert("actual_firing_angle", _model_outputs->actual_firing_angle);
        entry.signal_registry.insert("is_enabled", _model_outputs->is_enabled);
        entry.signal_registry.insert("is_synchronized", _model_outputs->is_synchronized);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("min_firing_angle", _model_parameters->min_firing_angle);
        entry.parameter_registry.insert("max_firing_angle", _model_parameters->max_firing_angle);
        entry.parameter_registry.insert("pulse_width", _model_parameters->pulse_width);
        entry.parameter_registry.insert("deadtime", _model_parameters->deadtime);
        entry.parameter_registry.insert("nominal_grid_frequency", _model_parameters->nominal_grid_frequency);

        registry.insert("sfc_grid_side_modulator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
