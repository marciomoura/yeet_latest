#pragma once

#include "sfc_integration/protection_severity.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_grid_side_protections_datamodel.hpp
 * @brief Datamodel for SFC Grid-Side Protection System
 *
 * Combines overcurrent, overvoltage, and undervoltage protections for the grid-side
 * of the Static Frequency Converter. All protections use latching trip behavior
 * with configurable thresholds and time delays.
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_protections_datamodel {
public:
    /**
     * @brief Input signals for grid-side protections
     */
    struct inputs {
        /// DC link current measurement [pu]
        signal_input<current_pu_t> dc_link_current_pu;

        /// Grid voltage magnitude [pu]
        signal_input<voltage_pu_t> grid_voltage_magnitude_pu;

        /// Release signal for protections (typically from state machine/supervisor)
        signal_input<bool> release_protections;

        /// Enclosure door is open (treated as a trip condition) [bool]
        signal_input<bool> door_is_open;

        /// External fault signal from hardware [bool]
        signal_input<bool> external_fault;

        /// Emergency off signal from hardware [bool]
        signal_input<bool> emergency_off;

        /// Machine-side protection tripped (received via fiber optics) [bool]
        signal_input<bool> machine_protection_tripped;
    };

    /**
     * @brief Output signals from grid-side protections
     */
    struct outputs {
        /// Any protection currently has an active trip (clears automatically when fault clears)
        signal_output<bool> has_active_trip;

        /// Any protection currently in alarm state (severity-dependent)
        signal_output<bool> has_active_alarm;

        /// Protection system inhibited (trip occurred, awaiting confirmation)
        signal_output<bool> is_inhibited;

        /// Overcurrent protection trip status
        signal_output<bool> overcurrent_tripped;

        /// Overcurrent protection alarm status (severity-dependent)
        signal_output<bool> overcurrent_alarmed;

        /// Overvoltage protection trip status
        signal_output<bool> overvoltage_tripped;

        /// Overvoltage protection alarm status (severity-dependent)
        signal_output<bool> overvoltage_alarmed;

        /// Undervoltage protection trip status
        signal_output<bool> undervoltage_tripped;

        /// Undervoltage protection alarm status (severity-dependent)
        signal_output<bool> undervoltage_alarmed;

        /// Overcurrent warning status
        signal_output<bool> overcurrent_warning;

        /// Overvoltage warning status
        signal_output<bool> overvoltage_warning;

        /// Undervoltage warning status
        signal_output<bool> undervoltage_warning;

        /// Communication loss protection trip status
        signal_output<bool> communication_loss_tripped;

        /// Communication loss protection alarm status (severity-dependent)
        signal_output<bool> communication_loss_alarmed;

        /// Communication loss protection warning status
        signal_output<bool> communication_loss_warning;

        /// Door open protection trip status (severity-dependent)
        signal_output<bool> door_open_tripped;

        /// Door open protection alarm status (severity-dependent)
        signal_output<bool> door_open_alarmed;

        /// External fault protection trip status (severity-dependent)
        signal_output<bool> external_fault_tripped;

        /// External fault protection alarm status (severity-dependent)
        signal_output<bool> external_fault_alarmed;

        /// Emergency off protection trip status (severity-dependent)
        signal_output<bool> emergency_off_tripped;

        /// Emergency off protection alarm status (severity-dependent)
        signal_output<bool> emergency_off_alarmed;

        /// Machine protection tripped status (received from machine-side via fiber optics)
        signal_output<bool> machine_protection_tripped;

        /// Machine protection alarm status (severity-dependent)
        signal_output<bool> machine_protection_alarmed;
    };

    /**
     * @brief Configuration parameters for grid-side protections
     */
    struct parameters {
        // ========== Per-Protection Severity Selection ==========
        /// Severity for overcurrent detection (default: trip)
        parameter<protection_severity> overcurrent_severity{protection_severity::trip};

        /// Severity for overvoltage detection (default: trip)
        parameter<protection_severity> overvoltage_severity{protection_severity::trip};

        /// Severity for undervoltage detection (default: trip)
        parameter<protection_severity> undervoltage_severity{protection_severity::trip};

        /// Severity for communication loss detection (default: trip)
        parameter<protection_severity> communication_loss_severity{protection_severity::trip};

        /// Severity for door open detection (default: trip)
        parameter<protection_severity> door_open_severity{protection_severity::trip};

        /// Severity for external fault detection (default: trip)
        parameter<protection_severity> external_fault_severity{protection_severity::trip};

        /// Severity for emergency off detection (default: trip)
        parameter<protection_severity> emergency_off_severity{protection_severity::trip};

        /// Severity for machine-side protection tripped (default: trip)
        parameter<protection_severity> machine_protection_severity{protection_severity::trip};

        // ========== Overcurrent Protection Parameters ==========
        /// Overcurrent trip threshold [pu]
        parameter<current_pu_t> overcurrent_trip_threshold{current_pu_t{1.5f}};

        /// Overcurrent warning threshold [pu]
        parameter<current_pu_t> overcurrent_warning_threshold{current_pu_t{1.2f}};

        /// Overcurrent trip delay [s]
        parameter<duration_t> overcurrent_trip_delay{duration_t{0.05}};

        /// Overcurrent warning delay [s]
        parameter<duration_t> overcurrent_warning_delay{duration_t{0.01}};

        // ========== Overvoltage Protection Parameters ==========
        /// Overvoltage trip threshold [pu]
        parameter<voltage_pu_t> overvoltage_trip_threshold{voltage_pu_t{1.2f}};

        /// Overvoltage warning threshold [pu]
        parameter<voltage_pu_t> overvoltage_warning_threshold{voltage_pu_t{1.1f}};

        /// Overvoltage trip delay [s]
        parameter<duration_t> overvoltage_trip_delay{duration_t{ms(500.0f)}};

        /// Overvoltage warning delay [s]
        parameter<duration_t> overvoltage_warning_delay{duration_t{ms(100.0f)}};

        // ========== Undervoltage Protection Parameters ==========
        /// Undervoltage trip threshold [pu]
        parameter<voltage_pu_t> undervoltage_trip_threshold{voltage_pu_t{0.70f}};

        /// Undervoltage warning threshold [pu]
        parameter<voltage_pu_t> undervoltage_warning_threshold{voltage_pu_t{0.85f}};

        /// Undervoltage trip delay [s]
        parameter<duration_t> undervoltage_trip_delay{duration_t{ms(100.0f)}};

        /// Undervoltage warning delay [s]
        parameter<duration_t> undervoltage_warning_delay{duration_t{ms(100.0f)}};

        // ========== Communication Watchdog Parameters ==========
        /// Communication timeout threshold [s]
        /// Default: 100ms
        parameter<duration_t> comm_timeout{duration_t{ms(100.0f)}};

        /// Communication reset threshold [s]
        /// Default: 5ms
        parameter<duration_t> comm_reset{duration_t{ms(5.0f)}};

        // ========== Protection Module Control ==========
        /// Master enable/disable for entire protection module
        parameter<bool> protection_module_enabled{false};

        /// Confirm/acknowledge protection trips (rising edge clears is_inhibited)
        parameter<bool> confirm_trips{false};

        // ========== Manual Reset Override (for testing) ==========
        /// Force reset all protections (for testing)
        parameter<bool> force_reset_all{false};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_protections_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "protections";
        // Per-protection severity parameters
        handler.register_parameter(component_name, "overcurrent_severity",
            yeet::utils::make_enum_accessor(_model_parameters->overcurrent_severity));
        handler.register_parameter(component_name, "overvoltage_severity",
            yeet::utils::make_enum_accessor(_model_parameters->overvoltage_severity));
        handler.register_parameter(component_name, "undervoltage_severity",
            yeet::utils::make_enum_accessor(_model_parameters->undervoltage_severity));
        handler.register_parameter(component_name, "communication_loss_severity",
            yeet::utils::make_enum_accessor(_model_parameters->communication_loss_severity));
        handler.register_parameter(component_name, "door_open_severity",
            yeet::utils::make_enum_accessor(_model_parameters->door_open_severity));
        handler.register_parameter(component_name, "external_fault_severity",
            yeet::utils::make_enum_accessor(_model_parameters->external_fault_severity));
        handler.register_parameter(component_name, "emergency_off_severity",
            yeet::utils::make_enum_accessor(_model_parameters->emergency_off_severity));
        handler.register_parameter(component_name, "machine_protection_severity",
            yeet::utils::make_enum_accessor(_model_parameters->machine_protection_severity));

        // Overcurrent parameters
        handler.register_parameter(component_name, "overcurrent_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overcurrent_trip_threshold));
        handler.register_parameter(component_name, "overcurrent_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overcurrent_warning_threshold));
        handler.register_parameter(component_name, "overcurrent_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->overcurrent_trip_delay));
        handler.register_parameter(component_name, "overcurrent_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->overcurrent_warning_delay));

        // Overvoltage parameters
        handler.register_parameter(component_name, "overvoltage_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overvoltage_trip_threshold));
        handler.register_parameter(component_name, "overvoltage_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overvoltage_warning_threshold));
        handler.register_parameter(component_name, "overvoltage_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->overvoltage_trip_delay));
        handler.register_parameter(component_name, "overvoltage_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->overvoltage_warning_delay));

        // Undervoltage parameters
        handler.register_parameter(component_name, "undervoltage_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->undervoltage_trip_threshold));
        handler.register_parameter(component_name, "undervoltage_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->undervoltage_warning_threshold));
        handler.register_parameter(component_name, "undervoltage_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->undervoltage_trip_delay));
        handler.register_parameter(component_name, "undervoltage_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->undervoltage_warning_delay));

        // Communication watchdog parameters
        handler.register_parameter(
            component_name, "comm_timeout", yeet::utils::make_units_accessor(_model_parameters->comm_timeout));
        handler.register_parameter(
            component_name, "comm_reset", yeet::utils::make_units_accessor(_model_parameters->comm_reset));

        // Protection module control
        handler.register_parameter(component_name, "protection_module_enabled",
            yeet::utils::make_bool_accessor(_model_parameters->protection_module_enabled));
        handler.register_parameter(
            component_name, "confirm_trips", yeet::utils::make_bool_accessor(_model_parameters->confirm_trips));

        // Manual reset override
        handler.register_parameter(
            component_name, "force_reset_all", yeet::utils::make_bool_accessor(_model_parameters->force_reset_all));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
