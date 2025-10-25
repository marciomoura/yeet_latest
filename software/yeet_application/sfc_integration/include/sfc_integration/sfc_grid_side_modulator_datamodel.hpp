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
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // angle_wrapped parameter - convert to/from radians
        auto& force_commutation_angle_param = _model_parameters->force_commutation_angle;
        entry.parameters.insert("force_commutation_angle",
            low_level_access(
                // Read: angle_wrapped -> float (radians)
                [&force_commutation_angle_param]() -> float {
                    return static_cast<float>(force_commutation_angle_param.read().get_radians());
                },
                // Write: float (radians) -> angle_wrapped
                [&force_commutation_angle_param](float value_rad) {
                    force_commutation_angle_param.write(angle_wrapped::from_radians(angle_t{value_rad}));
                }));

        // duration_t parameter - extract value in seconds
        auto& commutation_timeout_param = _model_parameters->commutation_timeout;
        entry.parameters.insert(
            "commutation_timeout", low_level_access(
                                       // Read: duration_t -> float (seconds)
                                       [&commutation_timeout_param]() -> float {
                                           return static_cast<float>(commutation_timeout_param.read().value());
                                       },
                                       // Write: float (seconds) -> duration_t
                                       [&commutation_timeout_param](float value_seconds) {
                                           commutation_timeout_param.write(duration_t{value_seconds});
                                       }));

        registry.insert("grid_side_modulator", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
