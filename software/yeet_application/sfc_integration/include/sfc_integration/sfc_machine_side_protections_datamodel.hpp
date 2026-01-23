#pragma once

#include "sfc/sfc_trip_source.hpp"
#include "sfc_integration/protection_severity.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_protections_datamodel.hpp
 * @brief Datamodel for SFC Machine-Side Protection System
 *
 * Combines V/Hz (flux) and stall protections for the machine-side
 * of the Static Frequency Converter. All protections use latching trip behavior
 * with configurable thresholds and time delays.
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_protections_datamodel {
public:
    /**
     * @brief Input signals for machine-side protections
     */
    struct inputs {
        /// DC link current measurement [pu]
        signal_input<current_pu_t> dc_link_current_pu;

        /// Machine voltage magnitude (RMS) [pu]
        signal_input<voltage_pu_t> machine_voltage_magnitude_pu;

        /// Machine electrical frequency [Hz]
        signal_input<frequency_t> machine_frequency;

        /// Machine electrical frequency [pu]
        signal_input<frequency_pu_t> machine_frequency_pu;

        /// Machine rotor speed [pu]
        signal_input<speed_pu_t> machine_speed;

        /// Startup active signal (for stall protection state-dependency)
        /// True during FLUX_BUILDUP and STARTING_PROCEDURE states
        signal_input<bool> startup_active;

        /// Closed-loop operation active signal (for VHz and overspeed protection)
        /// True when system is in CLOSED_LOOP_OPERATION with observer-based control
        signal_input<bool> closed_loop_active;

        /// Excitation system status (feedback from excitation controller)
        /// True when excitation is energized and operating normally
        signal_input<bool> excitation_is_on;

        /// Release signal for protections (typically from state machine/supervisor)
        signal_input<bool> release_protections;

        /// External fault signal from hardware [bool]
        signal_input<bool> external_fault;
    };

    /**
     * @brief Output signals from machine-side protections
     */
    struct outputs {
        /// Any protection currently has an active trip (clears automatically when fault clears)
        signal_output<bool> has_active_trip;

        /// Any protection currently in alarm state (severity-dependent)
        signal_output<bool> has_active_alarm;

        /// Protection system inhibited (trip occurred, awaiting confirmation)
        signal_output<bool> is_inhibited;

        /// Latch of the first trip source encountered
        signal_output<machine_trip_source> first_trip_source;

        /// V/Hz over-flux protection trip status
        signal_output<bool> over_flux_tripped;

        /// V/Hz over-flux protection alarm status (severity-dependent)
        signal_output<bool> over_flux_alarmed;

        /// V/Hz under-flux protection trip status
        signal_output<bool> under_flux_tripped;

        /// V/Hz under-flux protection alarm status (severity-dependent)
        signal_output<bool> under_flux_alarmed;

        /// Stall protection trip status
        signal_output<bool> stall_tripped;

        /// Stall protection alarm status (severity-dependent)
        signal_output<bool> stall_alarmed;

        /// V/Hz over-flux warning status
        signal_output<bool> over_flux_warning;

        /// V/Hz under-flux warning status
        signal_output<bool> under_flux_warning;

        /// Stall protection warning status
        signal_output<bool> stall_warning;

        /// Overspeed protection trip status
        signal_output<bool> overspeed_tripped;

        /// Overspeed protection alarm status (severity-dependent)
        signal_output<bool> overspeed_alarmed;

        /// Overspeed protection warning status
        signal_output<bool> overspeed_warning;

        /// Machine-side overvoltage protection trip status
        signal_output<bool> machine_overvoltage_tripped;

        /// Machine-side overvoltage protection alarm status (severity-dependent)
        signal_output<bool> machine_overvoltage_alarmed;

        /// Machine-side overvoltage protection warning status
        signal_output<bool> machine_overvoltage_warning;

        /// Communication loss protection trip status
        signal_output<bool> communication_loss_tripped;

        /// Communication loss protection alarm status (severity-dependent)
        signal_output<bool> communication_loss_alarmed;

        /// Communication loss protection warning status
        signal_output<bool> communication_loss_warning;

        /// Loss of excitation protection trip status
        signal_output<bool> loss_of_excitation_tripped;

        /// Loss of excitation protection alarm status (severity-dependent)
        signal_output<bool> loss_of_excitation_alarmed;

        /// Loss of excitation protection warning status
        signal_output<bool> loss_of_excitation_warning;

        /// External fault protection trip status
        signal_output<bool> external_fault_tripped;

        /// External fault protection alarm status (severity-dependent)
        signal_output<bool> external_fault_alarmed;
    };

    /**
     * @brief Configuration parameters for machine-side protections
     */
    struct parameters {
        // ========== Per-Protection Severity Selection ==========
        /// Severity for over-flux detection (default: trip)
        parameter<protection_severity> over_flux_severity{protection_severity::alarm_only};

        /// Severity for under-flux detection (default: trip)
        parameter<protection_severity> under_flux_severity{protection_severity::alarm_only};

        /// Severity for stall detection (default: trip)
        parameter<protection_severity> stall_severity{protection_severity::alarm_only};

        /// Severity for overspeed detection (default: trip)
        parameter<protection_severity> overspeed_severity{protection_severity::trip};

        /// Severity for machine overvoltage detection (default: trip)
        parameter<protection_severity> machine_overvoltage_severity{protection_severity::trip};

        /// Severity for communication loss detection (default: trip)
        parameter<protection_severity> communication_loss_severity{protection_severity::trip};

        /// Severity for loss of excitation detection (default: trip)
        parameter<protection_severity> loss_of_excitation_severity{protection_severity::trip};

        /// Severity for external fault detection (default: trip)
        parameter<protection_severity> external_fault_severity{protection_severity::trip};

        // ========== V/Hz Protection Parameters ==========
        /// Over-flux trip threshold [pu]
        parameter<flux_pu_t> over_flux_trip_threshold{flux_pu_t{10.10f}};

        /// Over-flux warning threshold [pu]
        parameter<flux_pu_t> over_flux_warning_threshold{flux_pu_t{1.05f}};

        /// Over-flux trip delay [s]
        parameter<duration_t> over_flux_trip_delay{duration_t{1.0}};

        /// Over-flux warning delay [s]
        parameter<duration_t> over_flux_warning_delay{duration_t{0.5}};

        /// Under-flux trip threshold [pu]
        parameter<flux_pu_t> under_flux_trip_threshold{flux_pu_t{0.080f}};

        /// Under-flux warning threshold [pu]
        parameter<flux_pu_t> under_flux_warning_threshold{flux_pu_t{0.90f}};

        /// Under-flux trip delay [s]
        parameter<duration_t> under_flux_trip_delay{duration_t{5.0}};

        /// Under-flux warning delay [s]
        parameter<duration_t> under_flux_warning_delay{duration_t{2.0}};

        // ========== Stall Protection Parameters ==========
        /// Stall current threshold [pu]
        parameter<current_pu_t> stall_current_threshold{current_pu_t{1.3f}};

        /// Stall speed threshold [pu]
        parameter<speed_pu_t> stall_speed_threshold{speed_pu_t{0.10f}};

        /// Stall trip delay [s]
        parameter<duration_t> stall_trip_delay{duration_t{3.0}};

        /// Stall warning delay [s]
        parameter<duration_t> stall_warning_delay{duration_t{1.0}};

        // ========== Overspeed Protection Parameters ==========
        /// Overspeed trip threshold [pu]
        parameter<speed_pu_t> overspeed_trip_threshold{speed_pu_t{1.10f}};

        /// Overspeed warning threshold [pu]
        parameter<speed_pu_t> overspeed_warning_threshold{speed_pu_t{1.05f}};

        /// Overspeed trip delay [s]
        parameter<duration_t> overspeed_trip_delay{duration_t{0.05}};

        /// Overspeed warning delay [s]
        parameter<duration_t> overspeed_warning_delay{duration_t{0.1}};

        // ========== Machine-Side Overvoltage Protection Parameters ==========
        /// Machine-side overvoltage trip threshold [pu]
        parameter<voltage_pu_t> machine_overvoltage_trip_threshold{voltage_pu_t{1.10f}};

        /// Machine-side overvoltage warning threshold [pu]
        parameter<voltage_pu_t> machine_overvoltage_warning_threshold{voltage_pu_t{1.05f}};

        /// Machine-side overvoltage trip delay [s]
        parameter<duration_t> machine_overvoltage_trip_delay{duration_t{0.05}};

        /// Machine-side overvoltage warning delay [s]
        parameter<duration_t> machine_overvoltage_warning_delay{duration_t{0.1}};

        // ========== Communication Watchdog Parameters ==========
        /// Communication timeout threshold [s]
        /// Default: 100ms
        parameter<duration_t> comm_timeout{duration_t{0.100}};

        /// Communication reset threshold [s]
        /// Default: 5ms
        parameter<duration_t> comm_reset{duration_t{0.005}};

        // ========== Loss of Excitation Protection Parameters ==========
        /// Loss of excitation trip threshold [real_t]
        parameter<real_t> loss_of_excitation_trip_threshold{real_t{0.5f}};

        /// Loss of excitation warning threshold [real_t]
        parameter<real_t> loss_of_excitation_warning_threshold{real_t{0.6f}};

        /// Loss of excitation trip delay [s]
        /// Default: 50ms (fast detection for critical fault)
        parameter<duration_t> loss_of_excitation_trip_delay{duration_t{0.05}};

        /// Loss of excitation warning delay [s]
        parameter<duration_t> loss_of_excitation_warning_delay{duration_t{0.2}};

        // ========== External Fault Parameters ==========
        /// External fault trip delay [s]
        parameter<duration_t> external_fault_trip_delay{duration_t{0.0}};

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
    sfc_machine_side_protections_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
        handler.register_parameter(component_name, "over_flux_severity",
            yeet::utils::make_enum_accessor(_model_parameters->over_flux_severity));
        handler.register_parameter(component_name, "under_flux_severity",
            yeet::utils::make_enum_accessor(_model_parameters->under_flux_severity));
        handler.register_parameter(
            component_name, "stall_severity", yeet::utils::make_enum_accessor(_model_parameters->stall_severity));
        handler.register_parameter(component_name, "overspeed_severity",
            yeet::utils::make_enum_accessor(_model_parameters->overspeed_severity));
        handler.register_parameter(component_name, "machine_overvoltage_severity",
            yeet::utils::make_enum_accessor(_model_parameters->machine_overvoltage_severity));
        handler.register_parameter(component_name, "communication_loss_severity",
            yeet::utils::make_enum_accessor(_model_parameters->communication_loss_severity));
        handler.register_parameter(component_name, "loss_of_excitation_severity",
            yeet::utils::make_enum_accessor(_model_parameters->loss_of_excitation_severity));
        handler.register_parameter(component_name, "external_fault_severity",
            yeet::utils::make_enum_accessor(_model_parameters->external_fault_severity));

        // V/Hz protection parameters
        handler.register_parameter(component_name, "over_flux_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->over_flux_trip_threshold));
        handler.register_parameter(component_name, "over_flux_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->over_flux_warning_threshold));
        handler.register_parameter(component_name, "over_flux_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->over_flux_trip_delay));
        handler.register_parameter(component_name, "over_flux_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->over_flux_warning_delay));
        handler.register_parameter(component_name, "under_flux_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->under_flux_trip_threshold));
        handler.register_parameter(component_name, "under_flux_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->under_flux_warning_threshold));
        handler.register_parameter(component_name, "under_flux_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->under_flux_trip_delay));
        handler.register_parameter(component_name, "under_flux_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->under_flux_warning_delay));

        // Stall protection parameters
        handler.register_parameter(component_name, "stall_current_threshold",
            yeet::utils::make_units_accessor(_model_parameters->stall_current_threshold));
        handler.register_parameter(component_name, "stall_speed_threshold",
            yeet::utils::make_units_accessor(_model_parameters->stall_speed_threshold));
        handler.register_parameter(
            component_name, "stall_trip_delay", yeet::utils::make_units_accessor(_model_parameters->stall_trip_delay));
        handler.register_parameter(component_name, "stall_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->stall_warning_delay));

        // Overspeed protection parameters
        handler.register_parameter(component_name, "overspeed_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overspeed_trip_threshold));
        handler.register_parameter(component_name, "overspeed_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overspeed_warning_threshold));
        handler.register_parameter(component_name, "overspeed_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->overspeed_trip_delay));
        handler.register_parameter(component_name, "overspeed_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->overspeed_warning_delay));

        // Machine-side overvoltage protection parameters
        handler.register_parameter(component_name, "machine_overvoltage_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->machine_overvoltage_trip_threshold));
        handler.register_parameter(component_name, "machine_overvoltage_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->machine_overvoltage_warning_threshold));
        handler.register_parameter(component_name, "machine_overvoltage_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->machine_overvoltage_trip_delay));
        handler.register_parameter(component_name, "machine_overvoltage_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->machine_overvoltage_warning_delay));

        // Communication watchdog parameters
        handler.register_parameter(
            component_name, "comm_timeout", yeet::utils::make_units_accessor(_model_parameters->comm_timeout));
        handler.register_parameter(
            component_name, "comm_reset", yeet::utils::make_units_accessor(_model_parameters->comm_reset));

        // Loss of excitation protection parameters
        handler.register_parameter(component_name, "loss_of_excitation_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->loss_of_excitation_trip_threshold));
        handler.register_parameter(component_name, "loss_of_excitation_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->loss_of_excitation_warning_threshold));
        handler.register_parameter(component_name, "loss_of_excitation_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->loss_of_excitation_trip_delay));
        handler.register_parameter(component_name, "loss_of_excitation_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->loss_of_excitation_warning_delay));

        // External fault protection parameters
        handler.register_parameter(component_name, "external_fault_trip_delay",
            yeet::utils::make_units_accessor(_model_parameters->external_fault_trip_delay));

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
