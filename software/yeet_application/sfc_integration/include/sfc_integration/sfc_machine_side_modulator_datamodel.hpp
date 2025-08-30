#pragma once

#include <cstdint>

#include "sfc/sfc_machine_side_modulator.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"

/**
 * @file sfc_machine_side_modulator_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Modulator component
 *
 * The machine-side modulator initiates forced commutation events and manages
 * firing angle selection based on operating mode. It coordinates with the
 * grid-side modulator through a handshake protocol for commutation at low speeds.
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_modulator_datamodel {
public:
    /**
     * @brief Input signals for the machine-side modulator component
     */
    struct inputs {
        /// System enable command
        signal_input<bool> enable;

        /// Operating mode selection from external mode detector
        signal_input<sfc_operating_mode_t> operating_mode;

        /// Acknowledgment signal from grid-side modulator
        signal_input<bool> force_commutation_acknowledged;

        /// Absolute rotor electrical position for sector tracking
        signal_input<angle_wrapped> machine_side_angle;

        /// Machine-side firing angle
        signal_input<angle_wrapped> firing_angle;
    };

    /**
     * @brief Output signals from the machine-side modulator component
     */
    struct outputs {
        /// Firing angle output to the pulse generator
        /// Range: [0.0, π] radians
        signal_output<angle_wrapped> firing_angle;

        /// Pulse release control signal to pulse generator
        signal_output<bool> release_pulses;

        /// Request signal to grid-side modulator for forced commutation
        signal_output<bool> request_force_commutation;

        /// Current commutation state for diagnostics
        signal_output<machine_side_commutation_state_t> commutation_state;

        /// Current rotor sector (1-6) for diagnostics
        signal_output<std::uint8_t> current_rotor_sector;
    };

    /**
     * @brief Configuration parameters for the machine-side modulator
     */
    struct parameters {
        /// Fixed firing angle for pulse mode operation
        /// Range: [0.0, π] radians
        /// Default: π radians (180°)
        parameter<angle_wrapped> pulse_mode_firing_angle = angle_wrapped(pi);  // 180°
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_modulator_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
