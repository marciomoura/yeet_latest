#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"  // for pulses
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_measurements_datamodel.hpp
 * @brief Integrated datamodel for SFC Machine-Side Measurements with filtering
 *
 * This component handles both raw measurement conditioning and signal filtering
 * for machine-side electrical and mechanical quantities.
 */
namespace yeet::application::sfc::integration {

class sfc_machine_side_measurements_ideal_datamodel {
public:
    /**
     * @brief Input signals for the machine-side measurements component
     */
    struct inputs {
        /// Machine-side voltage measurements (line-to-neutral) from plant model.
        signal_input<abc<voltage_t>> machine_voltage_abc;

        /// Machine-side current measurements (line-to-neutral) from plant model.
        signal_input<abc<current_t>> machine_current_abc;

        /// Machine-side pulse signals from plant model.
        signal_input<control::six_pulse_generator::pulses> machine_pulses;

        /// Machine-side electrical frequency from plant model.
        signal_input<frequency_t> machine_frequency;

        /// Machine-side electrical angle from plant model.
        signal_input<angle_wrapped> machine_reference_angle;
    };

    /**
     * @brief Output signals from the machine-side measurements component
     */
    struct outputs {
        /// Machine-side speed (per unit)
        signal_output<speed_pu_t> machine_speed;

        /// Machine-side voltage measurements in alphabeta frame
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta;

        /// Filtered machine-side voltage measurements in alphabeta frame
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta_filtered;

        /// Machine-side voltage measurements in abc frame
        signal_output<abc<voltage_pu_t>> machine_voltage_abc;

        /// Unmasked machine-side voltage measurements in abc frame
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_unmasked;

        /// Filtered machine-side voltage measurements in abc frame
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_filtered;

        /// Unmasked machine-side current measurements in abc frame
        signal_output<voltage_pu_t> machine_voltage_magnitude;

        /// Filtered machine-side voltage magnitude
        signal_output<voltage_pu_t> machine_voltage_magnitude_filtered;

        /// Machine-side current measurements in abc frame
        signal_output<abc<current_pu_t>> machine_current_abc;

        ///  Indicates if the machine is currently in a commutation event
        signal_output<bool> is_commuting;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {
        parameter<frequency_t> voltage_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> current_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> voltage_magnitude_lpf_cutoff_frequency{frequency_t{10.0}};
        parameter<voltage_pu_t> commutation_start_threshold{voltage_pu_t{0.1f}};
        parameter<voltage_pu_t> commutation_end_threshold{voltage_pu_t{0.02f}};
        // ADC conversion parameters (secondary side)
        parameter<int> voltage_adc_bits{16};
        parameter<real_t> voltage_adc_full_scale{real_t{10.0}};  // +/- full scale volts at card input
        // Transducer gain for documentation / future reconstruction
        parameter<real_t> voltage_transducer_gain{real_t{3.5 / 100.0}};  // V_out / V_primary_LL
        parameter<real_t> pt_ratio{real_t{1.0}};                         // Primary LL voltage / secondary LL voltage
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_measurements_ideal_datamodel(
        inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // frequency_t parameters - convert to/from Hz
        entry.parameters.insert("voltage_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_lpf_cutoff_frequency));
        entry.parameters.insert("current_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->current_lpf_cutoff_frequency));
        entry.parameters.insert("voltage_magnitude_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_magnitude_lpf_cutoff_frequency));

        // voltage_pu_t parameters - convert to/from per-unit
        entry.parameters.insert("commutation_start_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_start_threshold));
        entry.parameters.insert("commutation_end_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_end_threshold));

        // int and real_t parameters - use default accessor
        entry.parameters.insert("voltage_adc_bits", _model_parameters->voltage_adc_bits);
        entry.parameters.insert("voltage_adc_full_scale", _model_parameters->voltage_adc_full_scale);
        entry.parameters.insert("voltage_transducer_gain", _model_parameters->voltage_transducer_gain);
        entry.parameters.insert("pt_ratio", _model_parameters->pt_ratio);

        registry.insert("machine_side_measurements_ideal", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

class sfc_machine_side_measurements_datamodel {
public:
    /**
     * @brief Input signals for the machine-side measurements component
     */
    struct inputs {
        signal_input<control::adc_counts> v_ab_counts;
        signal_input<control::adc_counts> v_bc_counts;
        signal_input<control::six_pulse_generator::pulses> machine_pulses;
        signal_input<frequency_t> machine_frequency;
        signal_input<angle_wrapped> machine_reference_angle;
    };

    /**
     * @brief Output signals from the machine-side measurements component
     */
    struct outputs {
        signal_output<speed_pu_t> machine_speed;
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta;
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta_filtered;
        signal_output<abc<voltage_pu_t>> machine_voltage_abc;
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_unmasked;
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_filtered;
        signal_output<voltage_pu_t> machine_voltage_magnitude;
        signal_output<voltage_pu_t> machine_voltage_magnitude_filtered;
        signal_output<bool> is_commuting;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {
        parameter<frequency_t> voltage_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> current_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> voltage_magnitude_lpf_cutoff_frequency{frequency_t{10.0}};
        parameter<voltage_pu_t> commutation_start_threshold{voltage_pu_t{0.1f}};
        parameter<voltage_pu_t> commutation_end_threshold{voltage_pu_t{0.02f}};
        parameter<int> voltage_adc_bits{16};
        parameter<real_t> voltage_adc_full_scale{real_t{10.0}};
        parameter<real_t> voltage_transducer_gain{real_t{3.5 / 100.0}};
        parameter<real_t> pt_ratio{real_t{1.0}};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // frequency_t parameters - convert to/from Hz
        entry.parameters.insert("voltage_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_lpf_cutoff_frequency));
        entry.parameters.insert("current_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->current_lpf_cutoff_frequency));
        entry.parameters.insert("voltage_magnitude_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_magnitude_lpf_cutoff_frequency));

        // voltage_pu_t parameters - convert to/from per-unit
        entry.parameters.insert("commutation_start_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_start_threshold));
        entry.parameters.insert("commutation_end_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_end_threshold));

        // int and real_t parameters - use default accessor
        entry.parameters.insert("voltage_adc_bits", _model_parameters->voltage_adc_bits);
        entry.parameters.insert("voltage_adc_full_scale", _model_parameters->voltage_adc_full_scale);
        entry.parameters.insert("voltage_transducer_gain", _model_parameters->voltage_transducer_gain);
        entry.parameters.insert("pt_ratio", _model_parameters->pt_ratio);

        registry.insert("machine_side_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
