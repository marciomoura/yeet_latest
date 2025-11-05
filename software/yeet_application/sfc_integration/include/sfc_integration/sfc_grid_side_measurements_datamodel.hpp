#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"  // for pulses
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

class sfc_grid_side_measurements_ideal_datamodel {
public:
    /**
     * @brief Input signals for the grid-side measurements component
     */
    struct inputs {
        /// Raw analog grid-side voltage measurements (line-to-neutral) from plant model.
        signal_input<abc<voltage_t>> grid_voltage_abc;

        /// Raw analog grid-side current measurements from plant model.
        signal_input<abc<current_t>> grid_current_abc;

        /// Firing pulses from the six-pulse generator.
        signal_input<control::six_pulse_generator::pulses> grid_pulses;

        /// Grid frequency measurement input.
        signal_input<frequency_t> grid_frequency;

        /// Grid side angle measurement input.
        signal_input<angle_wrapped> grid_angle;
    };

    /**
     * @brief Output signals from the grid-side measurements component
     */
    struct outputs {
        /// Measured grid frequency in per unit.
        signal_output<frequency_pu_t> grid_frequency;

        /// Measured grid voltage in alpha-beta coordinates.
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta;

        /// Measured grid voltage in alpha-beta coordinates.
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta_filtered;

        /// Measured grid current in abc coordinates.
        signal_output<abc<voltage_pu_t>> grid_voltage_abc;

        /// Measured grid voltage in abc coordinates (unmasked).
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_unmasked;

        /// Measured grid voltage in abc coordinates (filtered).
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_filtered;

        /// Measured grid current in abc coordinates.
        signal_output<abc<current_pu_t>> grid_current_abc;

        /// Measured grid current in dq coordinates.
        signal_output<voltage_pu_t> grid_voltage_magnitude;

        /// Measured grid voltage magnitude (filtered).
        signal_output<voltage_pu_t> grid_voltage_magnitude_filtered;

        /// Commutation status signal.
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
        // ADC conversion (secondary side) parameters added to enable parameter-driven configuration
        parameter<int> voltage_adc_bits{16};
        parameter<real_t> voltage_adc_full_scale{real_t{10.0}};  // +/- full scale volts (card input)
        parameter<int> current_adc_bits{16};
        parameter<real_t> current_adc_full_scale{real_t{10.0}};  // +/- full scale volts (card input)
        // Transducer gains (secondary volts per primary unit) for documentation / potential future use
        parameter<real_t> voltage_transducer_gain{real_t{3.5 / 100.0}};  // V_out / V_primary_LL
        parameter<real_t> current_transducer_gain{real_t{1.0}};          // V_out / A_primary
        // Transformer ratios (Primary/Secondary). Default 1 (no scaling)
        parameter<real_t> pt_ratio{real_t{1.0}};  // Primary line-line voltage / secondary line-line voltage
        parameter<real_t> ct_ratio{real_t{1.0}};  // Primary current / secondary current
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_measurements_ideal_datamodel(
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

        entry.parameters.insert("voltage_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_lpf_cutoff_frequency));
        entry.parameters.insert("current_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->current_lpf_cutoff_frequency));
        entry.parameters.insert("voltage_magnitude_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_magnitude_lpf_cutoff_frequency));
        entry.parameters.insert("commutation_start_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_start_threshold));
        entry.parameters.insert("commutation_end_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_end_threshold));
        entry.parameters.insert("voltage_adc_bits", _model_parameters->voltage_adc_bits);
        entry.parameters.insert("voltage_adc_full_scale", _model_parameters->voltage_adc_full_scale);
        entry.parameters.insert("current_adc_bits", _model_parameters->current_adc_bits);
        entry.parameters.insert("current_adc_full_scale", _model_parameters->current_adc_full_scale);
        entry.parameters.insert("voltage_transducer_gain", _model_parameters->voltage_transducer_gain);
        entry.parameters.insert("current_transducer_gain", _model_parameters->current_transducer_gain);
        entry.parameters.insert("pt_ratio", _model_parameters->pt_ratio);
        entry.parameters.insert("ct_ratio", _model_parameters->ct_ratio);

        registry.insert("grid_side_measurements_ideal", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

class sfc_grid_side_measurements_datamodel {
public:
    /**
     * @brief Input signals for the grid-side measurements component
     */
    struct inputs {
        signal_input<control::adc_counts> v_ab_counts;
        signal_input<control::adc_counts> v_bc_counts;
        signal_input<control::adc_counts> i_a_counts;
        signal_input<control::adc_counts> i_c_counts;
        signal_input<control::six_pulse_generator::pulses> grid_pulses;
        signal_input<frequency_t> grid_frequency;
        signal_input<angle_wrapped> grid_angle;
    };

    /**
     * @brief Output signals from the grid-side measurements component
     */
    struct outputs {
        signal_output<frequency_pu_t> grid_frequency;
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta;
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta_filtered;
        signal_output<abc<voltage_pu_t>> grid_voltage_abc;
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_unmasked;
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_filtered;
        signal_output<abc<current_pu_t>> grid_current_abc;
        signal_output<voltage_pu_t> grid_voltage_magnitude;
        signal_output<voltage_pu_t> grid_voltage_magnitude_filtered;
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
        // ADC conversion (secondary side) parameters
        parameter<int> voltage_adc_bits{16};
        parameter<real_t> voltage_adc_full_scale{real_t{10.0}};
        parameter<int> current_adc_bits{16};
        parameter<real_t> current_adc_full_scale{real_t{10.0}};
        parameter<real_t> voltage_transducer_gain{real_t{3.5 / 100.0}};
        parameter<real_t> current_transducer_gain{real_t{1.0}};
        parameter<real_t> pt_ratio{real_t{1.0}};
        parameter<real_t> ct_ratio{real_t{1.0}};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
        entry.parameters.insert("current_adc_bits", _model_parameters->current_adc_bits);
        entry.parameters.insert("current_adc_full_scale", _model_parameters->current_adc_full_scale);
        entry.parameters.insert("voltage_transducer_gain", _model_parameters->voltage_transducer_gain);
        entry.parameters.insert("current_transducer_gain", _model_parameters->current_transducer_gain);
        entry.parameters.insert("pt_ratio", _model_parameters->pt_ratio);
        entry.parameters.insert("ct_ratio", _model_parameters->ct_ratio);

        registry.insert("grid_side_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
