#pragma once

#include <chrono>

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_integrator_supervisor_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Integrator Supervisor component
 *
 * The machine-side integrator supervisor centralizes freeze/preset/trip logic for both
 * the speed controller and torque controller. It collects freeze requests from:
 * - Grid-side supervisor (via fiber optics)
 * - Torque controller saturation (anti-windup)
 * - Local emergency stop
 *
 * Then coordinates unified freeze outputs to both controllers.
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_integrator_supervisor_datamodel {
public:
    /**
     * @brief Input signals for the machine-side integrator supervisor
     */
    struct inputs {
        /// Pulse release command from grid-side (via fiber optics)
        signal_input<bool> grid_side_release_pulses;

        /// Saturation signal from torque controller (anti-windup freeze)
        signal_input<bool> torque_saturated;

        /// Trip signal from machine-side protections (V/Hz, stall, communication loss)
        signal_input<bool> protection_shutdown_request;
    };

    /**
     * @brief Output signals from the machine-side integrator supervisor
     */
    struct outputs {
        /// Unified freeze command to both speed and torque controller integrators
        signal_output<bool> freeze_integrator;

        /// Preset request to both controllers (each controller presets internally)
        signal_output<bool> preset_request;

        /// Enable for both controllers to release pulses
        signal_output<bool> release_pulses;

        /// Status: fault condition active (propagated back to grid-side via fiber optics)
        signal_output<bool> request_shutdown;
    };

    /**
     * @brief Configuration parameters for the machine-side integrator supervisor
     */
    struct parameters {
        /// Override freeze signal for testing [bool]
        parameter<bool> override_freeze{false};

        /// Forced freeze value when override is active [bool]
        parameter<bool> freeze_value{false};

        /// Override release pulses signal for testing [bool]
        parameter<bool> override_release_pulses{false};

        /// Forced release pulses value when override is active [bool]
        parameter<bool> release_pulses_value{false};

        /// Override shutdown signal for testing [bool]
        parameter<bool> override_is_tripped{false};

        /// Forced shutdown value when override is active [bool]
        parameter<bool> is_tripped_value{false};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_integrator_supervisor_datamodel(
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

        entry.parameters.insert("override_freeze", yeet::utils::make_bool_accessor(_model_parameters->override_freeze));
        entry.parameters.insert("freeze_value", yeet::utils::make_bool_accessor(_model_parameters->freeze_value));
        entry.parameters.insert(
            "override_release_pulses", yeet::utils::make_bool_accessor(_model_parameters->override_release_pulses));
        entry.parameters.insert(
            "release_pulses_value", yeet::utils::make_bool_accessor(_model_parameters->release_pulses_value));
        entry.parameters.insert(
            "override_is_tripped", yeet::utils::make_bool_accessor(_model_parameters->override_is_tripped));
        entry.parameters.insert(
            "is_tripped_value", yeet::utils::make_bool_accessor(_model_parameters->is_tripped_value));

        registry.insert("integrator_supervisor", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
