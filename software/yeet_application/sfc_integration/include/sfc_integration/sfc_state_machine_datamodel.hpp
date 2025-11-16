#pragma once

#include "sfc/sfc_state_machine.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_state_machine_datamodel.hpp
 * @brief Datamodel specification for SFC State Machine component
 */

namespace yeet::application::sfc::integration {

class sfc_state_machine_datamodel {
public:
    /**
     * @brief Input signals for the state machine component
     */
    struct inputs {
        /// Auxiliaries power systems operational [bool]
        signal_input<bool> auxiliaries_are_on;

        /// Main circuit breakers closed and energized [bool]
        signal_input<bool> breakers_are_on;

        /// Start command [bool]
        signal_input<bool> start_command;

        /// Stop command [bool]
        signal_input<bool> stop_command;

        /// Emergency shutdown request [bool]
        signal_input<bool> shutdown_request;

        /// Current has been extinguished [bool]
        signal_input<bool> is_current_extinguished;

        /// Protection system inhibited (trip pending alarm confirmation) [bool]
        signal_input<bool> protection_inhibited;

        /// Any protection currently detecting active fault condition (NOT latched) [bool]
        signal_input<bool> any_protection_fault_active;
    };

    /**
     * @brief Output signals from the state machine component
     */
    struct outputs {
        /// Enable firing pulse generation [bool]
        signal_output<bool> release_pulses;

        /// Preset request signal to trigger controller preset [bool]
        signal_output<bool> preset_request;

        /// Request to extinguish DC link current [bool]
        signal_output<bool> request_extinguish_dc_current;

        /// Current state identifier for monitoring
        signal_output<sfc::sfc_state_id> current_state_id;

        /// State transition flag (true when transitioned in this cycle) [bool]
        signal_output<bool> has_transitioned;

        /// Enable protection release mechanism (true when not in IDLE state) [bool]
        signal_output<bool> release_protections;

        /// Trip acknowledged by state machine (rising edge on protection_inhibited) [bool]
        signal_output<bool> trip_acknowledged;

        /// Auxiliaries power systems operational [bool]
        signal_output<bool> auxiliaries_are_on;

        /// Main circuit breakers closed and energized [bool]
        signal_output<bool> breakers_are_on;

        /// Start command [bool]
        signal_output<bool> start_command;

        /// Stop command [bool]
        signal_output<bool> stop_command;

        /// Emergency shutdown request [bool]
        signal_output<bool> shutdown_request;

        /// Current has been extinguished [bool]
        signal_output<bool> is_current_extinguished;
    };

    /**
     * @brief Configuration parameters for the state machine
     */
    struct parameters {
        // ==================== Digital Input Overrides ====================
        // These parameters allow forcing digital inputs during testing/commissioning
        // When override_enable is true, the override_value is used instead of hardware input
        // Default: All overrides DISABLED to use actual input signals

        /// Enable override for auxiliaries_are_on input
        parameter<bool> override_enable_auxiliaries_are_on{false};
        /// Override value for auxiliaries_are_on (when override enabled)
        parameter<bool> override_value_auxiliaries_are_on{false};

        /// Enable override for breakers_are_on input
        parameter<bool> override_enable_breakers_are_on{false};
        /// Override value for breakers_are_on (when override enabled)
        parameter<bool> override_value_breakers_are_on{false};

        /// Enable override for start_command input
        parameter<bool> override_enable_start_command{false};
        /// Override value for start_command (when override enabled)
        parameter<bool> override_value_start_command{false};

        /// Enable override for stop_command input
        parameter<bool> override_enable_stop_command{false};
        /// Override value for stop_command (when override enabled)
        parameter<bool> override_value_stop_command{false};

        /// Enable override for shutdown_request input
        parameter<bool> override_enable_shutdown_request{false};
        /// Override value for shutdown_request (when override enabled)
        parameter<bool> override_value_shutdown_request{false};

        /// Enable override for is_current_extinguished input
        parameter<bool> override_enable_is_current_extinguished{false};
        /// Override value for is_current_extinguished (when override enabled)
        parameter<bool> override_value_is_current_extinguished{false};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_state_machine_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register digital input override parameters using boolean accessor
        entry.parameters.insert("override_enable_auxiliaries_are_on",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_auxiliaries_are_on));
        entry.parameters.insert("override_value_auxiliaries_are_on",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_auxiliaries_are_on));

        entry.parameters.insert("override_enable_breakers_are_on",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_breakers_are_on));
        entry.parameters.insert("override_value_breakers_are_on",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_breakers_are_on));

        entry.parameters.insert("override_enable_start_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_start_command));
        entry.parameters.insert("override_value_start_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_start_command));

        entry.parameters.insert("override_enable_stop_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_stop_command));
        entry.parameters.insert("override_value_stop_command",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_stop_command));

        entry.parameters.insert("override_enable_shutdown_request",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_shutdown_request));
        entry.parameters.insert("override_value_shutdown_request",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_shutdown_request));

        entry.parameters.insert("override_enable_is_current_extinguished",
            yeet::utils::make_bool_accessor(_model_parameters->override_enable_is_current_extinguished));
        entry.parameters.insert("override_value_is_current_extinguished",
            yeet::utils::make_bool_accessor(_model_parameters->override_value_is_current_extinguished));

        registry.insert("state_machine", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration