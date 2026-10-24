#pragma once

#include "sfc/sfc_state_machine.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
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

        /// Cooling fans are running feedback [bool]
        signal_input<bool> fans_are_running;

        /// Medium voltage breaker closed and energized [bool]
        signal_input<bool> mv_breaker_is_closed;

        /// Start command [bool]
        signal_input<bool> start_command;

        /// Stop command [bool]
        signal_input<bool> stop_command;

        /// Current has been extinguished [bool]
        signal_input<bool> is_current_extinguished;

        /// Protection system inhibited (trip pending alarm confirmation) [bool]
        signal_input<bool> protection_inhibited;

        /// Any protection currently detecting active fault condition (NOT latched) [bool]
        signal_input<bool> any_protection_fault_active;

        /// Protection relay indicates MV breaker is tripping [bool]
        signal_input<bool> mv_breaker_is_tripping;

        /// External fault input (ORs to protection trip) [bool]
        signal_input<bool> external_fault;

        /// Emergency off button pressed [bool]
        signal_input<bool> emergency_off;

        /// Enclosure door is open (prevents breaker close) [bool]
        signal_input<bool> door_is_open;

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

        /// Enable grid-side protection release mechanism (true when breakers are on) [bool]
        signal_output<bool> release_grid_protections;

        /// Enable machine-side protection release mechanism (via fiber optics, true when breakers are on) [bool]
        signal_output<bool> release_machine_protections;

        /// Auxiliaries power systems operational [bool]
        signal_output<bool> auxiliaries_are_on;

        /// Cooling fans are running feedback [bool]
        signal_output<bool> fans_are_running;

        /// Medium voltage breaker closed and energized [bool]
        signal_output<bool> mv_breaker_is_closed;

        /// Start command [bool]
        signal_output<bool> start_command;

        /// Stop command [bool]
        signal_output<bool> stop_command;

        /// Current has been extinguished [bool]
        signal_output<bool> is_current_extinguished;

        /// MV breaker close command (level-high when breaker should be closed) [bool]
        signal_output<bool> mv_breaker_close;

        /// MV breaker open command (level-high when breaker should be open) [bool]
        signal_output<bool> mv_breaker_open;

        /// Command to energize auxiliaries (level-high). Must go low on trips/shutdown. [bool]
        signal_output<bool> auxiliaries_on_command;
    };

    /**
     * @brief Configuration parameters for the state machine
     */
    struct parameters {
        // ==================== Operator Commands ====================
        /// Turn on command - energizes the system (IDLE -> AUXILIARIES_ON)
        parameter<bool> turn_on_command{false};

        /// Start command - starts operation (OPERATION_OFF -> OPERATION_ON)
        parameter<bool> start_command{false};

        /// Stop command - stops operation (triggers shutdown)
        parameter<bool> stop_command{false};

        // ==================== Digital Input Overrides ====================
        // These parameters allow forcing digital inputs during testing/commissioning
        // When <signal>_override is true, <signal>_override_value is used instead of hardware input
        // Default: All overrides DISABLED to use actual input signals

        /// Enable override for auxiliaries_are_on input
        parameter<bool> auxiliaries_are_on_override{false};
        /// Override value for auxiliaries_are_on (when override enabled)
        parameter<bool> auxiliaries_are_on_override_value{false};

        /// Enable override for fans_are_running input
        parameter<bool> fans_are_running_override{false};
        /// Override value for fans_are_running (when override enabled)
        parameter<bool> fans_are_running_override_value{false};

        /// Enable override for mv_breaker_is_closed input
        parameter<bool> mv_breaker_is_closed_override{false};
        /// Override value for mv_breaker_is_closed (when override enabled)
        parameter<bool> mv_breaker_is_closed_override_value{false};

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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "state_machine";
        // Register digital input override parameters using boolean accessor
        handler.register_parameter(component_name, "auxiliaries_are_on_override",
            yeet::utils::make_bool_accessor(_model_parameters->auxiliaries_are_on_override));
        handler.register_parameter(component_name, "auxiliaries_are_on_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->auxiliaries_are_on_override_value));

        handler.register_parameter(component_name, "fans_are_running_override",
            yeet::utils::make_bool_accessor(_model_parameters->fans_are_running_override));
        handler.register_parameter(component_name, "fans_are_running_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->fans_are_running_override_value));

        handler.register_parameter(component_name, "mv_breaker_is_closed_override",
            yeet::utils::make_bool_accessor(_model_parameters->mv_breaker_is_closed_override));
        handler.register_parameter(component_name, "mv_breaker_is_closed_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->mv_breaker_is_closed_override_value));

        handler.register_parameter(
            component_name, "turn_on_command", yeet::utils::make_bool_accessor(_model_parameters->turn_on_command));
        handler.register_parameter(
            component_name, "start_command", yeet::utils::make_bool_accessor(_model_parameters->start_command));
        handler.register_parameter(
            component_name, "stop_command", yeet::utils::make_bool_accessor(_model_parameters->stop_command));

        handler.register_parameter(component_name, "is_current_extinguished_override",
            yeet::utils::make_bool_accessor(_model_parameters->is_current_extinguished_override));
        handler.register_parameter(component_name, "is_current_extinguished_override_value",
            yeet::utils::make_bool_accessor(_model_parameters->is_current_extinguished_override_value));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
