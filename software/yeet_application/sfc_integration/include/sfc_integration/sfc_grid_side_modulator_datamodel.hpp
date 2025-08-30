#pragma once

#include "sfc/sfc_grid_side_modulator.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_grid_side_modulator_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Modulator component
 *
 * The grid-side modulator acts as the master controller for forced commutation,
 * coordinating the handshake protocol with the machine-side modulator and
 * managing the override of the current controller's firing angle during
 * commutation events.
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_modulator_datamodel {
public:
    /**
     * @brief Input signals for the grid-side modulator component
     */
    struct inputs {
        /// Request signal from machine-side modulator to initiate forced commutation
        signal_input<bool> request_force_commutation;

        /// Signal from zero current detector indicating DC link current is zero
        signal_input<bool> dc_link_current_is_zero;

        /// Firing angle reference from the main DC link current controller
        /// Range: [0.0, π] radians
        signal_input<angle_wrapped> firing_angle;

        /// System enable command
        signal_input<bool> enable;

        /// Operating mode selector for load commutated vs pulse mode
        signal_input<sfc_operating_mode_t> operating_mode;
    };

    /**
     * @brief Output signals from the grid-side modulator component
     */
    struct outputs {
        /// Firing angle output to the pulse generator
        /// Range: [0.0, π] radians
        signal_output<angle_wrapped> firing_angle;

        /// Acknowledgment signal to machine-side modulator (one-cycle pulse)
        signal_output<bool> force_commutation_acknowledged;

        /// Request signal to hold main PI controller integrator in reset
        signal_output<bool> is_requesting_main_pi_reset;

        /// Current modulator state for diagnostics
        /// 0 = natural_commutation, 1 = forced_commutation
        signal_output<grid_side_modulator_state_t> modulator_state;
    };

    /**
     * @brief Configuration parameters for the grid-side modulator
     */
    struct parameters {
        /// Fixed firing angle for forced commutation (deep inversion)
        /// Range: [2π/3, π] radians (120° to 180°)
        /// Default: 5π/6 radians (150°)
        parameter<angle_wrapped> force_commutation_angle = angle_wrapped(2.6180);  // 150°

        /// Timeout duration for forced commutation (future safety feature)
        /// Range: [0.01, 1.0] seconds
        /// Default: 0.1 seconds
        parameter<duration_t> commutation_timeout = duration_t(0.1);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_modulator_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        // Note: Implementation commented out due to compilation issues with signal registry
        // TODO: Fix signal registry insert method compatibility

        // model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // // Register input signals
        // entry.signal_registry.insert("request_force_commutation", _model_inputs->request_force_commutation);
        // entry.signal_registry.insert("dc_link_current_is_zero", _model_inputs->dc_link_current_is_zero);
        // entry.signal_registry.insert("firing_angle_from_current_controller",
        //                              _model_inputs->firing_angle_from_current_controller);
        // entry.signal_registry.insert("enable", _model_inputs->enable);

        // // Register output signals
        // entry.signal_registry.insert("firing_angle_to_pulse_generator",
        //                              _model_outputs->firing_angle_to_pulse_generator);
        // entry.signal_registry.insert("force_commutation_acknowledged",
        //                              _model_outputs->force_commutation_acknowledged);
        // entry.signal_registry.insert("is_requesting_main_pi_reset",
        //                              _model_outputs->is_requesting_main_pi_reset);
        // entry.signal_registry.insert("modulator_state", _model_outputs->modulator_state);

        // // Register parameters
        // entry.parameter_registry.insert("force_commutation_angle",
        //                                 _model_parameters->force_commutation_angle);
        // entry.parameter_registry.insert("commutation_timeout",
        //                                 _model_parameters->commutation_timeout);

        // registry.insert("sfc_grid_side_modulator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
