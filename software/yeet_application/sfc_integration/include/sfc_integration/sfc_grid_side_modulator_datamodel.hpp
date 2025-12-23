#pragma once

#include "sfc/sfc_grid_side_modulator.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
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
        /// Request signal from machine-side modulator to initiate forced commutation [bool]
        signal_input<bool> request_force_commutation;

        /// Request signal to extinguish dc-link current [bool]
        signal_input<bool> request_extinguish_dc_current;

        /// Signal indicating DC link current is zero [bool]
        signal_input<bool> current_is_zero;

        /// Firing angle reference from current controller [rad]
        signal_input<angle_wrapped> firing_angle;

        /// Operating mode selector (natural vs pulse mode)
        signal_input<sfc_operating_mode_t> operating_mode;

        /// External shutdown/trip signal [bool]
        signal_input<bool> shutdown_request;

        /// Fast protection trip signal (direct from protection system) [bool]
        signal_input<bool> protection_trip;

        /// Release pulses control signal from integrator supervisor [bool]
        signal_input<bool> release_pulses;
    };

    /**
     * @brief Output signals from the grid-side modulator component
     */
    struct outputs {
        /// Pulse release control signal to pulse generator [bool]
        signal_output<bool> release_pulses;

        /// Firing angle output to pulse generator [rad]
        signal_output<angle_wrapped> firing_angle;

        /// Acknowledgment signal to machine-side modulator [bool]
        signal_output<bool> force_commutation_acknowledged;

        /// Request signal to freeze current controller integrator [bool]
        signal_output<bool> freeze_integrator;

        /// Current modulator state for diagnostics
        signal_output<grid_side_modulator_state_t> modulator_state;
    };

    /**
     * @brief Configuration parameters for the grid-side modulator
     */
    struct parameters {
        /// Firing angle for forced commutation [rad]
        parameter<angle_wrapped> force_commutation_angle = angle_wrapped(2.6180);

        /// Timeout duration for forced commutation [s]
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "modulator";
        handler.register_parameter(component_name, "force_commutation_angle",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->force_commutation_angle));
        handler.register_parameter(component_name, "commutation_timeout",
            yeet::utils::make_units_accessor(_model_parameters->commutation_timeout));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
