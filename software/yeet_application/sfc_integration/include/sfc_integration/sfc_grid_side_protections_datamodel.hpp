#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
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

        /// Trip acknowledged by state machine (allows clearing of latched trip)
        signal_input<bool> trip_acknowledged;
    };

    /**
     * @brief Output signals from grid-side protections
     */
    struct outputs {
        /// Combined trip status (any protection tripped)
        signal_output<bool> is_tripped;

        /// Protection system inhibited (trip occurred, awaiting alarm confirmation)
        signal_output<bool> is_inhibited;

        /// Any individual protection currently detecting active fault condition (NOT latched)
        /// Clears immediately when fault condition clears, unlike is_tripped which latches
        signal_output<bool> any_fault_active;

        /// Overcurrent protection trip status
        signal_output<bool> overcurrent_tripped;

        /// Overvoltage protection trip status
        signal_output<bool> overvoltage_tripped;

        /// Undervoltage protection trip status
        signal_output<bool> undervoltage_tripped;

        /// Overcurrent warning status
        signal_output<bool> overcurrent_warning;

        /// Overvoltage warning status
        signal_output<bool> overvoltage_warning;

        /// Undervoltage warning status
        signal_output<bool> undervoltage_warning;

        /// Communication loss protection trip status
        signal_output<bool> communication_loss_tripped;

        /// Communication loss protection warning status
        signal_output<bool> communication_loss_warning;
    };

    /**
     * @brief Configuration parameters for grid-side protections
     */
    struct parameters {
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
        parameter<voltage_pu_t> overvoltage_trip_threshold{voltage_pu_t{1.25f}};

        /// Overvoltage warning threshold [pu]
        parameter<voltage_pu_t> overvoltage_warning_threshold{voltage_pu_t{1.15f}};

        /// Overvoltage trip delay [s]
        parameter<duration_t> overvoltage_trip_delay{duration_t{0.1}};

        /// Overvoltage warning delay [s]
        parameter<duration_t> overvoltage_warning_delay{duration_t{0.02}};

        // ========== Undervoltage Protection Parameters ==========
        /// Undervoltage trip threshold [pu]
        parameter<voltage_pu_t> undervoltage_trip_threshold{voltage_pu_t{0.70f}};

        /// Undervoltage warning threshold [pu]
        parameter<voltage_pu_t> undervoltage_warning_threshold{voltage_pu_t{0.85f}};

        /// Undervoltage trip delay [s]
        parameter<duration_t> undervoltage_trip_delay{duration_t{0.5}};

        /// Undervoltage warning delay [s]
        parameter<duration_t> undervoltage_warning_delay{duration_t{0.1}};

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
    sfc_grid_side_protections_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Overcurrent parameters
        entry.parameters.insert("overcurrent_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overcurrent_trip_threshold));
        entry.parameters.insert("overcurrent_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overcurrent_warning_threshold));
        entry.parameters.insert(
            "overcurrent_trip_delay", yeet::utils::make_units_accessor(_model_parameters->overcurrent_trip_delay));
        entry.parameters.insert("overcurrent_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->overcurrent_warning_delay));

        // Overvoltage parameters
        entry.parameters.insert("overvoltage_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overvoltage_trip_threshold));
        entry.parameters.insert("overvoltage_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->overvoltage_warning_threshold));
        entry.parameters.insert(
            "overvoltage_trip_delay", yeet::utils::make_units_accessor(_model_parameters->overvoltage_trip_delay));
        entry.parameters.insert("overvoltage_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->overvoltage_warning_delay));

        // Undervoltage parameters
        entry.parameters.insert("undervoltage_trip_threshold",
            yeet::utils::make_units_accessor(_model_parameters->undervoltage_trip_threshold));
        entry.parameters.insert("undervoltage_warning_threshold",
            yeet::utils::make_units_accessor(_model_parameters->undervoltage_warning_threshold));
        entry.parameters.insert(
            "undervoltage_trip_delay", yeet::utils::make_units_accessor(_model_parameters->undervoltage_trip_delay));
        entry.parameters.insert("undervoltage_warning_delay",
            yeet::utils::make_units_accessor(_model_parameters->undervoltage_warning_delay));

        // Communication watchdog parameters
        entry.parameters.insert("comm_timeout", yeet::utils::make_units_accessor(_model_parameters->comm_timeout));
        entry.parameters.insert("comm_reset", yeet::utils::make_units_accessor(_model_parameters->comm_reset));

        // Protection module control
        entry.parameters.insert(
            "protection_module_enabled", yeet::utils::make_bool_accessor(_model_parameters->protection_module_enabled));
        entry.parameters.insert("confirm_alarms", yeet::utils::make_bool_accessor(_model_parameters->confirm_alarms));

        // Manual reset override
        entry.parameters.insert("force_reset_all", yeet::utils::make_bool_accessor(_model_parameters->force_reset_all));

        registry.insert("grid_side_protections", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
