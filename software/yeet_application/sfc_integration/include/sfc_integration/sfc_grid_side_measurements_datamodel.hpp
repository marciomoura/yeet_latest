#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_grid_side_measurements_datamodel.hpp
 * @brief Integrated datamodel for SFC Grid-Side Measurements with filtering
 *
 * This component handles both raw measurement conditioning and signal filtering
 * to provide clean, filtered measurement signals for control algorithms.
 */

namespace yeet::application::sfc::integration {

class sfc_grid_side_measurements_datamodel {
public:
    /**
     * @brief Input signals for the grid-side measurements component
     */
    struct inputs {
        /// Three-phase line-to-line voltages (raw sensor values)
        /// Range: [0, 20000] V typical for medium voltage applications
        signal_input<abc<voltage_t>> line_voltages_raw;

        /// Three-phase line currents (raw sensor values)
        /// Range: [0, 5000] A typical for high power applications
        signal_input<abc<current_t>> line_currents_raw;

        /// DC link voltage measurement (raw)
        /// Range: [0, 15000] V typical
        signal_input<voltage_t> dc_link_voltage_raw;

        /// Grid frequency for adaptive filtering
        /// Range: [45.0, 65.0] Hz
        signal_input<frequency_t> grid_frequency;

        /// Sensor health status
        signal_input<bool> sensor_health_ok;

        /// Filtering enable command
        signal_input<bool> filtering_enable;
    };

    /**
     * @brief Output signals from the grid-side measurements component
     */
    struct outputs {
        /// Filtered grid voltage in alpha-beta stationary frame (per-unit)
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta_pu;

        /// Filtered grid current in alpha-beta stationary frame (per-unit)
        signal_output<alphabeta<current_pu_t>> grid_current_alphabeta_pu;

        /// Filtered three-phase voltages (per-unit)
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_pu;

        /// Filtered three-phase currents (per-unit)
        signal_output<abc<current_pu_t>> grid_current_abc_pu;

        /// Grid voltage magnitude (RMS, filtered, per-unit)
        signal_output<voltage_pu_t> grid_voltage_magnitude_pu;

        /// Grid current magnitude (RMS, filtered, per-unit)
        signal_output<current_pu_t> grid_current_magnitude_pu;

        /// Filtered DC voltage in per-unit
        signal_output<voltage_pu_t> dc_voltage_pu;

        /// Voltage harmonic distortion factor
        /// Range: [0.0, 1.0] (fraction)
        signal_output<real_t> voltage_thd;

        /// Current harmonic distortion factor
        /// Range: [0.0, 1.0] (fraction)
        signal_output<real_t> current_thd;

        /// Measurement validity status
        signal_output<bool> is_valid;

        /// Filtering status (active/inactive)
        signal_output<bool> filtering_active;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {
        // Scaling parameters
        parameter<real_t> voltage_scale_factor = real_t(1.0);
        parameter<real_t> current_scale_factor = real_t(1.0);

        // Per-unit base values
        parameter<voltage_t> voltage_base = voltage_t(1000.0);
        parameter<current_t> current_base = current_t(1000.0);

        // Low-pass filtering parameters
        parameter<frequency_t> lpf_cutoff_frequency = frequency_t(200.0);
        parameter<int> lpf_filter_order = 4;

        // Harmonic filtering parameters
        parameter<frequency_t> notch_frequency_5th = frequency_t(250.0);   // 5th harmonic at 50Hz
        parameter<frequency_t> notch_frequency_7th = frequency_t(350.0);   // 7th harmonic at 50Hz
        parameter<frequency_t> notch_frequency_11th = frequency_t(550.0);  // 11th harmonic at 50Hz
        parameter<frequency_t> notch_frequency_13th = frequency_t(650.0);  // 13th harmonic at 50Hz

        // Notch filter quality factors
        parameter<real_t> notch_q_factor = real_t(10.0);

        // Anti-aliasing filter
        parameter<frequency_t> anti_aliasing_frequency = frequency_t(2000.0);

        // THD calculation parameters
        parameter<real_t> thd_calculation_window = real_t(0.02);  // 20ms window (1 cycle at 50Hz)
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("grid_voltage_alphabeta_pu", _model_outputs->grid_voltage_alphabeta_pu);
        entry.signal_registry.insert("grid_current_alphabeta_pu", _model_outputs->grid_current_alphabeta_pu);
        entry.signal_registry.insert("grid_voltage_abc_pu", _model_outputs->grid_voltage_abc_pu);
        entry.signal_registry.insert("grid_current_abc_pu", _model_outputs->grid_current_abc_pu);
        entry.signal_registry.insert("grid_voltage_magnitude_pu", _model_outputs->grid_voltage_magnitude_pu);
        entry.signal_registry.insert("grid_current_magnitude_pu", _model_outputs->grid_current_magnitude_pu);
        entry.signal_registry.insert("dc_voltage_pu", _model_outputs->dc_voltage_pu);
        entry.signal_registry.insert("voltage_thd", _model_outputs->voltage_thd);
        entry.signal_registry.insert("current_thd", _model_outputs->current_thd);
        entry.signal_registry.insert("is_valid", _model_outputs->is_valid);
        entry.signal_registry.insert("filtering_active", _model_outputs->filtering_active);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("voltage_scale_factor", _model_parameters->voltage_scale_factor);
        entry.parameter_registry.insert("current_scale_factor", _model_parameters->current_scale_factor);
        entry.parameter_registry.insert("voltage_base", _model_parameters->voltage_base);
        entry.parameter_registry.insert("current_base", _model_parameters->current_base);
        entry.parameter_registry.insert("lpf_cutoff_frequency", _model_parameters->lpf_cutoff_frequency);
        entry.parameter_registry.insert("lpf_filter_order", _model_parameters->lpf_filter_order);
        entry.parameter_registry.insert("notch_frequency_5th", _model_parameters->notch_frequency_5th);
        entry.parameter_registry.insert("notch_frequency_7th", _model_parameters->notch_frequency_7th);
        entry.parameter_registry.insert("notch_frequency_11th", _model_parameters->notch_frequency_11th);
        entry.parameter_registry.insert("notch_frequency_13th", _model_parameters->notch_frequency_13th);
        entry.parameter_registry.insert("notch_q_factor", _model_parameters->notch_q_factor);
        entry.parameter_registry.insert("anti_aliasing_frequency", _model_parameters->anti_aliasing_frequency);
        entry.parameter_registry.insert("thd_calculation_window", _model_parameters->thd_calculation_window);

        registry.insert("sfc_grid_side_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
