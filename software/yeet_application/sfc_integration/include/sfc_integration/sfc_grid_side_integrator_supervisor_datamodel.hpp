#pragma once

#include <chrono>

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_grid_side_integrator_supervisor_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Integrator Supervisor component
 *
 * The grid-side integrator supervisor centralizes freeze/preset/trip logic for the
 * current controller. It collects requests from the state machine, modulator, and
 * external shutdown signals, then coordinates a unified freeze output to the controller.
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_integrator_supervisor_datamodel {
public:
    /**
     * @brief Input signals for the grid-side integrator supervisor
     */
    struct inputs {
        /// Preset request from state machine (triggers component preset sequences)
        signal_input<bool> preset_request;

        /// Forced commutation freeze request from grid-side modulator
        signal_input<bool> forced_commutation_freeze_integrator;

        /// Trip/shutdown request from external IOS
        signal_input<bool> shutdown_request;

        /// Pulse release command from state machine
        signal_input<bool> state_machine_release_pulses;

        /// Signal from current controller indicating preset is complete
        signal_input<bool> preset_complete;
    };

    /**
     * @brief Output signals from the grid-side integrator supervisor
     */
    struct outputs {
        /// Unified freeze command to current controller integrator
        signal_output<bool> freeze_integrator;

        /// Preset request to current controller (controller presets internally)
        signal_output<bool> preset_request;

        /// Enable for current controller to release pulses
        signal_output<bool> release_pulses;

        /// Status: fault condition active (requires acknowledgment to clear)
        signal_output<bool> is_tripped;
    };

    /**
     * @brief Configuration parameters for the grid-side integrator supervisor
     */
    struct parameters {
        /// Override freeze signal for testing (when true, disables all freezes)
        parameter<bool> override_freeze{false};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_integrator_supervisor_datamodel(
        inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert("override_freeze", _model_parameters->override_freeze);

        registry.insert("grid_side_integrator_supervisor", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
