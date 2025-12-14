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
        signal_input<bool> mcb_is_closed;

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

        /// Protection relay indicates breaker is tripping [bool]
        signal_input<bool> breaker_is_tripping;

        /// Emergency off button pressed [bool]
        signal_input<bool> emergency_off;

        /// Enclosure door is open (prevents breaker close) [bool]
        signal_input<bool> door_is_open;

        /// Shutdown request from machine-side (via fiber link) [bool]
        signal_input<bool> machine_side_shutdown_request;

        /// Machine-side protections inhibited (awaiting confirmation) [bool]
        signal_input<bool> machine_side_protection_inhibited;

        /// Machine-side protections tripped (latched) [bool]
        signal_input<bool> machine_side_protection_tripped;
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
        signal_output<bool> mcb_is_closed;

        /// Start command [bool]
        signal_output<bool> start_command;

        /// Stop command [bool]
        signal_output<bool> stop_command;

        /// Emergency shutdown request [bool]
        signal_output<bool> shutdown_request;

        /// Current has been extinguished [bool]
        signal_output<bool> is_current_extinguished;

        /// Main circuit breaker close (level-high when breaker should be closed) [bool]
        signal_output<bool> mcb_close;

        /// Main circuit breaker open (level-high when breaker should be open) [bool]
        signal_output<bool> mcb_open;

        /// Command to energize auxiliaries (level-high). Must go low on trips/shutdown. [bool]
        signal_output<bool> auxiliaries_on_command;
    };

    /**
     * @brief Configuration parameters for the state machine
     */
    struct parameters {
        // ==================== Digital Input Overrides ====================
        // These parameters allow forcing digital inputs during testing/commissioning
        // When <signal>_override is true, <signal>_override_value is used instead of hardware input
        // Default: All overrides DISABLED to use actual input signals

        /// Enable override for auxiliaries_are_on input
        parameter<bool> auxiliaries_are_on_override{false};
        /// Override value for auxiliaries_are_on (when override enabled)
        parameter<bool> auxiliaries_are_on_override_value{false};

        /// Enable override for mcb_is_closed input
        parameter<bool> mcb_is_closed_override{false};
        /// Override value for mcb_is_closed (when override enabled)
        parameter<bool> mcb_is_closed_override_value{false};

        /// Enable override for start_command input
        parameter<bool> start_command_override{false};
        /// Override value for start_command (when override enabled)
        parameter<bool> start_command_override_value{false};

        /// Enable override for stop_command input
        parameter<bool> stop_command_override{false};
        /// Override value for stop_command (when override enabled)
        parameter<bool> stop_command_override_value{false};

        /// Enable override for shutdown_request input
        parameter<bool> shutdown_request_override{false};
        /// Override value for shutdown_request (when override enabled)
        parameter<bool> shutdown_request_override_value{false};

        /// Enable override for is_current_extinguished input
        parameter<bool> is_current_extinguished_override{false};
        /// Override value for is_current_extinguished (when override enabled)
        parameter<bool> is_current_extinguished_override_value{false};
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
        entry.parameters.insert("auxiliaries_are_on_override",
            yeet::utils::make_bool_accessor(_model_parameters->auxiliaries_are_on_override));
        entry.parameters.insert("auxiliaries_are_on_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->auxiliaries_are_on_override_value));

        entry.parameters.insert(
            "mcb_is_closed_override", yeet::utils::make_bool_accessor(_model_parameters->mcb_is_closed_override));
        entry.parameters.insert("mcb_is_closed_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->mcb_is_closed_override_value));

        entry.parameters.insert(
            "start_command_override", yeet::utils::make_bool_accessor(_model_parameters->start_command_override));
        entry.parameters.insert("start_command_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->start_command_override_value));

        entry.parameters.insert(
            "stop_command_override", yeet::utils::make_bool_accessor(_model_parameters->stop_command_override));
        entry.parameters.insert("stop_command_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->stop_command_override_value));

        entry.parameters.insert(
            "shutdown_request_override", yeet::utils::make_bool_accessor(_model_parameters->shutdown_request_override));
        entry.parameters.insert("shutdown_request_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->shutdown_request_override_value));

        entry.parameters.insert("is_current_extinguished_override",
            yeet::utils::make_bool_accessor(_model_parameters->is_current_extinguished_override));
        entry.parameters.insert("is_current_extinguished_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->is_current_extinguished_override_value));

        registry.insert("state_machine", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
