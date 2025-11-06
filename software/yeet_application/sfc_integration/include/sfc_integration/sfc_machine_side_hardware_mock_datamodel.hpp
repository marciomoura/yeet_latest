#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel specification for the Machine-Side Hardware Mock component.
 *
 * This defines the data contract (inputs, outputs, parameters) for simulating
 * the machine-side voltage measurement hardware, including ADCs.
 */
class sfc_machine_side_hardware_mock_datamodel {
public:
    /**
     * @brief Input signals for the machine-side hardware mock.
     */
    struct inputs {
        /// Raw analog machine-side voltage measurements (line-to-neutral) from plant model.
        signal_input<abc<voltage_t>> machine_voltage_abc;
    };

    /**
     * @brief Output signals from the machine-side hardware mock.
     */
    struct outputs {
        /// Quantized line-to-line voltage Vab counts from the mock ADC.
        signal_output<control::adc_counts> v_ab_counts;

        /// Quantized line-to-line voltage Vbc counts from the mock ADC.
        signal_output<control::adc_counts> v_bc_counts;
    };

    /**
     * @brief Configuration parameters for the machine-side hardware mock.
     */
    struct parameters {
        /// Voltage ADC gain.
        parameter<real_t> voltage_adc_gain = real_t(1.0);
        /// Voltage ADC anti-aliasing filter cutoff frequency (Hz).
        parameter<frequency_t> voltage_adc_cutoff_freq = frequency_t(2500.0);
        /// Voltage ADC resolution in bits.
        parameter<int> voltage_adc_bits = 12;
        /// Voltage ADC full-scale range.
        parameter<real_t> voltage_adc_full_scale = real_t(1.0);
        /// Potential transformer ratio (Primary_V / Secondary_V)
        parameter<real_t> pt_ratio = real_t(1.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_hardware_mock_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert(
            "voltage_adc_gain", yeet::utils::make_units_accessor(_model_parameters->voltage_adc_gain));
        entry.parameters.insert(
            "voltage_adc_cutoff_freq", yeet::utils::make_units_accessor(_model_parameters->voltage_adc_cutoff_freq));
        entry.parameters.insert(
            "voltage_adc_bits", yeet::utils::make_units_accessor(_model_parameters->voltage_adc_bits));
        entry.parameters.insert(
            "voltage_adc_full_scale", yeet::utils::make_units_accessor(_model_parameters->voltage_adc_full_scale));
        entry.parameters.insert("pt_ratio", yeet::utils::make_units_accessor(_model_parameters->pt_ratio));

        registry.insert("machine_side_hardware_mock", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration