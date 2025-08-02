#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_measurements_datamodel.hpp
 * @brief Integrated datamodel for SFC Machine-Side Measurements with filtering
 *
 * This component handles both raw measurement conditioning and signal filtering
 * for machine-side electrical and mechanical quantities.
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_measurements_datamodel {
public:
    /**
     * @brief Input signals for the machine-side measurements component
     */
    struct inputs {
        /// Three-phase machine voltages (raw sensor values)
        /// Range: [0, 5000] V typical for machine applications
        signal_input<abc<voltage_t>> machine_voltages_raw;

        /// Three-phase machine currents (raw sensor values)
        /// Range: [0, 2000] A typical for machine applications
        signal_input<abc<current_t>> machine_currents_raw;

        /// Machine speed measurement (raw)
        /// Range: [0, 4000] RPM typical for gas turbine applications
        signal_input<speed_t> machine_speed_raw;

        /// Rotor position measurement (raw)
        /// Range: [0, 2π] radians
        signal_input<angle<zero_to_two_pi>> rotor_position_raw;

        /// Machine frequency for adaptive filtering
        /// Range: [0, 100] Hz (variable frequency)
        signal_input<frequency_t> machine_frequency;

        /// Sensor health status
        signal_input<bool> sensor_health_ok;

        /// Filtering enable command
        signal_input<bool> filtering_enable;
    };

    /**
     * @brief Output signals from the machine-side measurements component
     */
    struct outputs {
        /// Filtered machine voltage in alpha-beta stationary frame (per-unit)
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta_pu;

        /// Filtered machine current in alpha-beta stationary frame (per-unit)
        signal_output<alphabeta<current_pu_t>> machine_current_alphabeta_pu;

        /// Filtered three-phase machine voltages (per-unit)
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_pu;

        /// Filtered three-phase machine currents (per-unit)
        signal_output<abc<current_pu_t>> machine_current_abc_pu;

        /// Filtered machine speed in per-unit
        signal_output<speed_pu_t> machine_speed_pu;

        /// Filtered rotor position (electrical angle)
        signal_output<angle<zero_to_two_pi>> rotor_position_electrical;

        /// Filtered machine voltage magnitude (per-unit)
        signal_output<voltage_pu_t> machine_voltage_magnitude_pu;

        /// Filtered machine current magnitude (per-unit)
        signal_output<current_pu_t> machine_current_magnitude_pu;

        /// Machine back-EMF magnitude (estimated, per-unit)
        signal_output<voltage_pu_t> back_emf_magnitude_pu;

        /// Current ripple factor
        /// Range: [0.0, 1.0] (fraction)
        signal_output<real_t> current_ripple_factor;

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
        parameter<real_t> speed_scale_factor = real_t(1.0);

        // Machine parameters
        parameter<real_t> pole_pairs = real_t(1.0);

        // Low-pass filtering parameters
        parameter<frequency_t> voltage_lpf_cutoff = frequency_t(500.0);
        parameter<frequency_t> current_lpf_cutoff = frequency_t(300.0);
        parameter<frequency_t> speed_lpf_cutoff = frequency_t(10.0);
        parameter<int> filter_order = 3;

        // High-pass filtering for DC offset removal
        parameter<frequency_t> current_hpf_cutoff = frequency_t(1.0);

        // Anti-aliasing filters
        parameter<frequency_t> anti_aliasing_frequency = frequency_t(2000.0);

        // Ripple calculation parameters
        parameter<real_t> ripple_calculation_window = real_t(0.01);  // 10ms window
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register outputs in signal registry
        // entry.signal_registry.insert("machine_voltage_alphabeta_pu", _model_outputs->machine_voltage_alphabeta_pu);
        // entry.signal_registry.insert("machine_current_alphabeta_pu", _model_outputs->machine_current_alphabeta_pu);
        // entry.signal_registry.insert("machine_voltage_abc_pu", _model_outputs->machine_voltage_abc_pu);
        // entry.signal_registry.insert("machine_current_abc_pu", _model_outputs->machine_current_abc_pu);
        // entry.signal_registry.insert("machine_speed_pu", _model_outputs->machine_speed_pu);
        // entry.signal_registry.insert("rotor_position_electrical", _model_outputs->rotor_position_electrical);
        // entry.signal_registry.insert("machine_voltage_magnitude_pu", _model_outputs->machine_voltage_magnitude_pu);
        // entry.signal_registry.insert("machine_current_magnitude_pu", _model_outputs->machine_current_magnitude_pu);
        // entry.signal_registry.insert("back_emf_magnitude_pu", _model_outputs->back_emf_magnitude_pu);
        // entry.signal_registry.insert("current_ripple_factor", _model_outputs->current_ripple_factor);
        // entry.signal_registry.insert("is_valid", _model_outputs->is_valid);
        // entry.signal_registry.insert("filtering_active", _model_outputs->filtering_active);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("voltage_scale_factor", _model_parameters->voltage_scale_factor);
        // entry.parameter_registry.insert("current_scale_factor", _model_parameters->current_scale_factor);
        // entry.parameter_registry.insert("speed_scale_factor", _model_parameters->speed_scale_factor);
        // entry.parameter_registry.insert("pole_pairs", _model_parameters->pole_pairs);
        // entry.parameter_registry.insert("voltage_lpf_cutoff", _model_parameters->voltage_lpf_cutoff);
        // entry.parameter_registry.insert("current_lpf_cutoff", _model_parameters->current_lpf_cutoff);
        // entry.parameter_registry.insert("speed_lpf_cutoff", _model_parameters->speed_lpf_cutoff);
        // entry.parameter_registry.insert("filter_order", _model_parameters->filter_order);
        // entry.parameter_registry.insert("current_hpf_cutoff", _model_parameters->current_hpf_cutoff);
        // entry.parameter_registry.insert("anti_aliasing_frequency", _model_parameters->anti_aliasing_frequency);
        // entry.parameter_registry.insert("ripple_calculation_window", _model_parameters->ripple_calculation_window);

        // registry.insert("sfc_machine_side_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
