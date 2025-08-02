#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_dc_link_measurements_datamodel.hpp
 * @brief Datamodel specification for SFC DC Link Measurements component
 */

namespace yeet::application::sfc::integration {

class sfc_dc_link_measurements_datamodel {
public:
    /**
     * @brief Input signals for the DC link measurements component
     */
    struct inputs {
        /// DC link voltage measurement (raw sensor value)
        /// Range: [0, 15000] V typical for high voltage DC links
        signal_input<voltage_t> dc_voltage;

        /// DC link current measurement (raw sensor value)
        /// Range: [-5000, 5000] A typical for high power applications
        signal_input<current_t> dc_current;

        /// Sensor health status
        signal_input<bool> sensor_health_ok;
    };

    /**
     * @brief Output signals from the DC link measurements component
     */
    struct outputs {
        /// DC voltage in per-unit
        signal_output<voltage_pu_t> dc_voltage_pu;

        /// DC current in per-unit
        signal_output<current_pu_t> dc_current_pu;

        /// Filtered DC voltage in per-unit (removes ripple)
        signal_output<voltage_pu_t> dc_voltage_filtered_pu;

        /// Filtered DC current in per-unit (removes ripple)
        signal_output<current_pu_t> dc_current_filtered_pu;

        /// Instantaneous DC power in per-unit
        signal_output<power_pu_t> dc_power_pu;

        /// Measurement validity status
        signal_output<bool> is_valid;
    };

    /**
     * @brief Configuration parameters for the DC link measurements
     */
    struct parameters {
        /// Voltage scaling factor from raw measurements
        parameter<real_t> voltage_scale_factor = real_t(1.0);

        /// Current scaling factor from raw measurements
        parameter<real_t> current_scale_factor = real_t(1.0);

        /// Low-pass filter cutoff frequency for ripple removal
        parameter<frequency_t> filter_cutoff_frequency = frequency_t(100.0);

        /// Voltage base for per-unit conversion
        parameter<voltage_t> voltage_base = voltage_t(1000.0);

        /// Current base for per-unit conversion
        parameter<current_t> current_base = current_t(1000.0);

        /// Power base for per-unit conversion (calculated from V_base * I_base)
        parameter<power_t> power_base = power_t(1000000.0);  // 1 MW
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_dc_link_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("dc_voltage_pu", _model_outputs->dc_voltage_pu);
        entry.signal_registry.insert("dc_current_pu", _model_outputs->dc_current_pu);
        entry.signal_registry.insert("dc_voltage_filtered_pu", _model_outputs->dc_voltage_filtered_pu);
        entry.signal_registry.insert("dc_current_filtered_pu", _model_outputs->dc_current_filtered_pu);
        entry.signal_registry.insert("dc_power_pu", _model_outputs->dc_power_pu);
        entry.signal_registry.insert("is_valid", _model_outputs->is_valid);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("voltage_scale_factor", _model_parameters->voltage_scale_factor);
        entry.parameter_registry.insert("current_scale_factor", _model_parameters->current_scale_factor);
        entry.parameter_registry.insert("filter_cutoff_frequency", _model_parameters->filter_cutoff_frequency);
        entry.parameter_registry.insert("voltage_base", _model_parameters->voltage_base);
        entry.parameter_registry.insert("current_base", _model_parameters->current_base);
        entry.parameter_registry.insert("power_base", _model_parameters->power_base);

        registry.insert("sfc_dc_link_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
