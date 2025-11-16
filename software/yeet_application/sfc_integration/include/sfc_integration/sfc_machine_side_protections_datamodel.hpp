#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
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

        /// Machine rotor speed [pu]
        signal_input<speed_pu_t> machine_speed;

        /// Startup active signal (for stall protection state-dependency)
        signal_input<bool> startup_active;

        /// Release signal for protections (typically from state machine/supervisor)
        signal_input<bool> release_protections;

        /// Trip acknowledged by state machine (allows clearing of latched trip)
        signal_input<bool> trip_acknowledged;
    };

    /**
     * @brief Output signals from machine-side protections
     */
    struct outputs {
        /// Combined trip status (any protection tripped)
        signal_output<bool> is_tripped;

        /// Protection system inhibited (trip occurred, awaiting alarm confirmation)
        signal_output<bool> is_inhibited;

        /// Any individual protection currently detecting active fault condition (NOT latched)
        /// Clears immediately when fault condition clears, unlike is_tripped which latches
        signal_output<bool> any_fault_active;

        /// V/Hz over-flux protection trip status
        signal_output<bool> over_flux_tripped;

        /// V/Hz under-flux protection trip status
        signal_output<bool> under_flux_tripped;

        /// Stall protection trip status
        signal_output<bool> stall_tripped;

        /// V/Hz over-flux warning status
        signal_output<bool> over_flux_warning;

        /// V/Hz under-flux warning status
        signal_output<bool> under_flux_warning;

        /// Stall protection warning status
        signal_output<bool> stall_warning;

        /// Communication loss protection trip status
        signal_output<bool> communication_loss_tripped;

        /// Communication loss protection warning status
        signal_output<bool> communication_loss_warning;
    };

    /**
     * @brief Configuration parameters for machine-side protections
     */
    struct parameters {
        // ========== V/Hz Protection Parameters ==========
        /// Rated V/Hz value for normalization [pu]
        parameter<flux_pu_t> rated_vhz{flux_pu_t{1.0f}};

        /// Over-flux trip threshold [pu]
        parameter<flux_pu_t> over_flux_trip_threshold{flux_pu_t{1.10f}};

        /// Over-flux warning threshold [pu]
        parameter<flux_pu_t> over_flux_warning_threshold{flux_pu_t{1.05f}};

        /// Over-flux trip delay [s]
        parameter<duration_t> over_flux_trip_delay{duration_t{1.0}};

        /// Over-flux warning delay [s]
        parameter<duration_t> over_flux_warning_delay{duration_t{0.5}};

        /// Under-flux trip threshold [pu]
        parameter<flux_pu_t> under_flux_trip_threshold{flux_pu_t{0.80f}};

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

        // ========== Communication Watchdog Parameters ==========
        /// Communication timeout threshold [s]
        /// Default: 50ms
        parameter<duration_t> comm_timeout{duration_t{0.050}};

        /// Communication reset threshold [s]
        /// Default: 5ms
        parameter<duration_t> comm_reset{duration_t{0.005}};

        // ========== Protection Module Control ==========
        /// Master enable/disable for entire protection module
        parameter<bool> protection_module_enabled{true};

        /// Confirm/acknowledge protection alarms (rising edge clears is_inhibited)
        parameter<bool> confirm_alarms{false};

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

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // V/Hz protection parameters
        entry.parameters.insert("rated_vhz", yeet::utils::make_units_accessor(_model_parameters->rated_vhz));
        entry.parameters.insert(
            "over_flux_trip_threshold", yeet::utils::make_units_accessor(_model_parameters->over_flux_trip_threshold));
        entry.parameters.insert("over_flux_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->over_flux_warning_threshold));
        entry.parameters.insert(
            "over_flux_trip_delay", yeet::utils::make_units_accessor(_model_parameters->over_flux_trip_delay));
        entry.parameters.insert(
            "over_flux_warning_delay", yeet::utils::make_units_accessor(_model_parameters->over_flux_warning_delay));
        entry.parameters.insert("under_flux_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->under_flux_trip_threshold));
        entry.parameters.insert("under_flux_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->under_flux_warning_threshold));
        entry.parameters.insert(
            "under_flux_trip_delay", yeet::utils::make_units_accessor(_model_parameters->under_flux_trip_delay));
        entry.parameters.insert(
            "under_flux_warning_delay", yeet::utils::make_units_accessor(_model_parameters->under_flux_warning_delay));

        // Stall protection parameters
        entry.parameters.insert(
            "stall_current_threshold", yeet::utils::make_units_accessor(_model_parameters->stall_current_threshold));
        entry.parameters.insert(
            "stall_speed_threshold", yeet::utils::make_units_accessor(_model_parameters->stall_speed_threshold));
        entry.parameters.insert(
            "stall_trip_delay", yeet::utils::make_units_accessor(_model_parameters->stall_trip_delay));
        entry.parameters.insert(
            "stall_warning_delay", yeet::utils::make_units_accessor(_model_parameters->stall_warning_delay));

        // Communication watchdog parameters
        entry.parameters.insert("comm_timeout", yeet::utils::make_units_accessor(_model_parameters->comm_timeout));
        entry.parameters.insert("comm_reset", yeet::utils::make_units_accessor(_model_parameters->comm_reset));

        // Protection module control
        entry.parameters.insert(
            "protection_module_enabled", yeet::utils::make_bool_accessor(_model_parameters->protection_module_enabled));
        entry.parameters.insert("confirm_alarms", yeet::utils::make_bool_accessor(_model_parameters->confirm_alarms));

        // Manual reset override
        entry.parameters.insert("force_reset_all", yeet::utils::make_bool_accessor(_model_parameters->force_reset_all));

        registry.insert("machine_side_protections", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
