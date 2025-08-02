#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"


/**
 * @file sfc_machine_side_modulator_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Modulator component
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_modulator_datamodel {
public:
    /**
     * @brief Input signals for the machine-side modulator component
     */
    struct inputs {
        /// Enable command for the modulator
        signal_input<bool> enable_command;

        /// Operating mode selection (pulse_mode, normal_mode, bypass_mode)
        signal_input<int> operating_mode;  // enum represented as int

        /// Rotor electrical position
        /// Range: [0.0, 2π] radians
        signal_input<angle_t> rotor_position_electrical;

        /// Firing angle from lookup table
        /// Range: [0.0, π] radians
        signal_input<angle_t> lookup_table_firing_angle;

        /// DC link current for advance angle calculation
        /// Range: [-1.5, 1.5] pu
        signal_input<current_pu_t> dc_link_current_pu;

        /// Commutation detection signal
        signal_input<bool> commutation_detected;
    };

    /**
     * @brief Output signals from the machine-side modulator component
     */
    struct outputs {
        /// Final calculated firing angle
        /// Range: [0.0, π] radians
        signal_output<angle_t> firing_angle;

        /// Six-pulse firing signals for machine-side thyristors
        signal_output<control::six_pulse_generator::pulses> firing_pulses;

        /// Active operating mode
        signal_output<int> active_operating_mode;

        /// Pulse mode firing angle (diagnostic)
        /// Range: [π/4, π/2] radians
        signal_output<angle_t> pulse_mode_firing_angle;

        /// Advance angle compensation
        /// Range: [0.0, π/6] radians
        signal_output<angle_t> advance_angle;

        /// Modulator enabled status
        signal_output<bool> is_enabled;

        /// Commutation failure detection
        signal_output<bool> has_commutation_failure;

        /// Next firing position prediction
        signal_output<angle_t> next_firing_position;
    };

    /**
     * @brief Configuration parameters for the machine-side modulator
     */
    struct parameters {
        /// Fixed firing angle for pulse mode operation
        /// Range: [π/4, π/2] radians
        parameter<angle_t> pulse_mode_firing_angle = angle_t(1.5708);  // π/2 = 90°

        /// Current threshold for advance angle activation
        /// Range: [0.1, 0.5] pu
        parameter<current_pu_t> advance_angle_current_threshold = current_pu_t(0.2);

        /// Maximum advance angle compensation
        /// Range: [0.0, π/6] radians
        parameter<angle_t> max_advance_angle = angle_t(0.5236);  // π/6 = 30°

        /// Firing pulse width
        /// Range: [π/36, π/6] radians
        parameter<angle_t> firing_pulse_width = angle_t(0.1745);  // π/18 = 10°

        /// Commutation failure timeout
        /// Range: [0.001, 0.01] s
        parameter<duration_t> commutation_timeout = duration_t(0.005);

        /// Minimum firing angle limit
        /// Range: [0.0, π/6] radians
        parameter<angle_t> min_firing_angle = angle_t(0.0);

        /// Maximum firing angle limit
        /// Range: [2π/3, π] radians
        parameter<angle_t> max_firing_angle = angle_t(3.1416);  // π = 180°
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_modulator_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("firing_angle", _model_outputs->firing_angle);
        entry.signal_registry.insert("firing_pulses", _model_outputs->firing_pulses);
        entry.signal_registry.insert("active_operating_mode", _model_outputs->active_operating_mode);
        entry.signal_registry.insert("pulse_mode_firing_angle", _model_outputs->pulse_mode_firing_angle);
        entry.signal_registry.insert("advance_angle", _model_outputs->advance_angle);
        entry.signal_registry.insert("is_enabled", _model_outputs->is_enabled);
        entry.signal_registry.insert("has_commutation_failure", _model_outputs->has_commutation_failure);
        entry.signal_registry.insert("next_firing_position", _model_outputs->next_firing_position);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("pulse_mode_firing_angle", _model_parameters->pulse_mode_firing_angle);
        entry.parameter_registry.insert("advance_angle_current_threshold",
                                        _model_parameters->advance_angle_current_threshold);
        entry.parameter_registry.insert("max_advance_angle", _model_parameters->max_advance_angle);
        entry.parameter_registry.insert("firing_pulse_width", _model_parameters->firing_pulse_width);
        entry.parameter_registry.insert("commutation_timeout", _model_parameters->commutation_timeout);
        entry.parameter_registry.insert("min_firing_angle", _model_parameters->min_firing_angle);
        entry.parameter_registry.insert("max_firing_angle", _model_parameters->max_firing_angle);

        registry.insert("sfc_machine_side_modulator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
