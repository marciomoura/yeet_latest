#pragma once

#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
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
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_raw;

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

        /// Commutation masking activity indicator
        signal_output<bool> is_masking;

        /// Last measured commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_last;

        /// Filtered commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_filtered;

        /// Line voltages (unmasked) [V]
        signal_output<line_voltage<voltage_t>> grid_voltage_line;

        /// Phase currents [A]
        signal_output<abc<current_t>> grid_current;

        /// Grid voltage AB at ADC card input [V]
        signal_output<voltage_t> voltage_ab_card;

        /// Grid current A at ADC card input [V]
        signal_output<voltage_t> current_a_card_voltage;

        /// Grid voltage AB at PT secondary [V]
        signal_output<voltage_t> voltage_ab_secondary;

        /// Grid current A at CT secondary [A]
        signal_output<current_t> current_a_secondary;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {
        parameter<frequency_t> voltage_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> current_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> voltage_magnitude_lpf_cutoff_frequency{frequency_t{10.0}};

        /// Cutoff frequency of the hardware low-pass filter for voltage measurements [Hz]
        parameter<frequency_t> voltage_compensation_lpf_cutoff_frequency_1{frequency_t{0.0}};

        /// Cutoff frequency of the second hardware low-pass filter stage [Hz]
        parameter<frequency_t> voltage_compensation_lpf_cutoff_frequency_2{frequency_t{0.0}};

        /// Number of series-connected hardware low-pass filters to compensate
        parameter<int> voltage_compensation_num_filters{0};

        /// Enable phase-shift compensation for the hardware filter
        parameter<bool> enable_voltage_compensation{false};

        /// Enable commutation notch masking
        parameter<bool> enable_commutation_masking{true};

        /// Rotation angle for voltage measurements [deg]
        parameter<angle_wrapped> voltage_measurement_rotation{angle_wrapped{0.0f}};

        parameter<voltage_pu_t> commutation_start_threshold{voltage_pu_t{0.1f}};
        parameter<voltage_pu_t> commutation_end_threshold{voltage_pu_t{0.02f}};

        /// Time constant for smoothing commutation notch duration [s]
        parameter<duration_t> commutation_notch_duration_filter_time_constant{duration_t{10e-3F}};
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "measurements_ideal";
        handler.register_parameter(component_name, "voltage_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "current_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->current_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "voltage_magnitude_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_magnitude_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "voltage_compensation_lpf_cutoff_frequency_1",
            yeet::utils::make_units_accessor(_model_parameters->voltage_compensation_lpf_cutoff_frequency_1));
        handler.register_parameter(component_name, "voltage_compensation_lpf_cutoff_frequency_2",
            yeet::utils::make_units_accessor(_model_parameters->voltage_compensation_lpf_cutoff_frequency_2));
        handler.register_parameter(component_name, "voltage_compensation_num_filters",
            yeet::utils::make_units_accessor(_model_parameters->voltage_compensation_num_filters));
        handler.register_parameter(component_name, "enable_voltage_compensation",
            yeet::utils::make_bool_accessor(_model_parameters->enable_voltage_compensation));
        handler.register_parameter(component_name, "enable_commutation_masking",
            yeet::utils::make_bool_accessor(_model_parameters->enable_commutation_masking));
        handler.register_parameter(component_name, "voltage_measurement_rotation",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->voltage_measurement_rotation));
        handler.register_parameter(component_name, "commutation_start_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_start_threshold));
        handler.register_parameter(component_name, "commutation_end_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_end_threshold));
        handler.register_parameter(component_name, "commutation_notch_duration_filter_time_constant",
            yeet::utils::make_units_accessor(_model_parameters->commutation_notch_duration_filter_time_constant));
        handler.register_parameter(
            component_name, "voltage_adc_bits", yeet::utils::make_units_accessor(_model_parameters->voltage_adc_bits));
        handler.register_parameter(component_name, "voltage_adc_full_scale",
            yeet::utils::make_units_accessor(_model_parameters->voltage_adc_full_scale));
        handler.register_parameter(
            component_name, "current_adc_bits", yeet::utils::make_units_accessor(_model_parameters->current_adc_bits));
        handler.register_parameter(component_name, "current_adc_full_scale",
            yeet::utils::make_units_accessor(_model_parameters->current_adc_full_scale));
        handler.register_parameter(component_name, "voltage_transducer_gain",
            yeet::utils::make_units_accessor(_model_parameters->voltage_transducer_gain));
        handler.register_parameter(component_name, "current_transducer_gain",
            yeet::utils::make_units_accessor(_model_parameters->current_transducer_gain));
        handler.register_parameter(
            component_name, "pt_ratio", yeet::utils::make_units_accessor(_model_parameters->pt_ratio));
        handler.register_parameter(
            component_name, "ct_ratio", yeet::utils::make_units_accessor(_model_parameters->ct_ratio));
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
        /// Grid voltage ADC counts (phase A to B) [counts]
        signal_input<control::adc_counts> v_ab_counts;

        /// Grid voltage ADC counts (phase B to C) [counts]
        signal_input<control::adc_counts> v_bc_counts;

        /// Grid current ADC counts (phase A) [counts]
        signal_input<control::adc_counts> i_a_counts;

        /// Grid current ADC counts (phase C) [counts]
        signal_input<control::adc_counts> i_c_counts;

        /// Firing pulses for commutation detection
        signal_input<control::six_pulse_generator::pulses> grid_pulses;

        /// Grid frequency estimate [Hz]
        signal_input<frequency_t> grid_frequency;

        /// Grid voltage phase angle [rad]
        signal_input<angle_wrapped> grid_angle;
    };

    /**
     * @brief Output signals from the grid-side measurements component
     */
    struct outputs {
        /// Grid frequency estimate [pu]
        signal_output<frequency_pu_t> grid_frequency;

        /// Grid voltage in alpha-beta coordinates [pu]
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta;

        /// Grid voltage in alpha-beta coordinates, filtered [pu]
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta_filtered;

        /// Grid voltage in abc coordinates [pu]
        signal_output<abc<voltage_pu_t>> grid_voltage_abc;

        /// Grid voltage in abc coordinates, unmasked [pu]
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_raw;

        /// Grid voltage in abc coordinates, filtered [pu]
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_filtered;

        /// Grid current in abc coordinates [pu]
        signal_output<abc<current_pu_t>> grid_current_abc;

        /// Grid voltage magnitude [pu]
        signal_output<voltage_pu_t> grid_voltage_magnitude;

        /// Grid voltage magnitude, filtered [pu]
        signal_output<voltage_pu_t> grid_voltage_magnitude_filtered;

        /// Commutation activity indicator
        signal_output<bool> is_commuting;

        /// Commutation masking activity indicator
        signal_output<bool> is_masking;

        /// Last measured commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_last;

        /// Filtered commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_filtered;

        /// Line voltages (unmasked) [V]
        signal_output<line_voltage<voltage_t>> grid_voltage_line;

        /// Phase currents [A]
        signal_output<abc<current_t>> grid_current;

        /// Grid voltage AB at ADC card input [V]
        signal_output<voltage_t> voltage_ab_card;

        /// Grid current A at ADC card input [V]
        signal_output<voltage_t> current_a_card_voltage;

        /// Grid voltage AB at PT secondary [V]
        signal_output<voltage_t> voltage_ab_secondary;

        /// Grid current A at CT secondary [A]
        signal_output<current_t> current_a_secondary;

        /// Peak ADC counts for Vab chan [counts]
        signal_output<real_t> v_ab_counts_peak;

        /// Peak ADC counts for Vbc chan [counts]
        signal_output<real_t> v_bc_counts_peak;

        /// Peak ADC counts for Ia chan [counts]
        signal_output<real_t> i_a_counts_peak;

        /// Peak ADC counts for Ic chan [counts]
        signal_output<real_t> i_c_counts_peak;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {
        /// Voltage low-pass filter cutoff frequency [Hz]
        parameter<frequency_t> voltage_lpf_cutoff_frequency{frequency_t{100.0}};

        /// Current low-pass filter cutoff frequency [Hz]
        parameter<frequency_t> current_lpf_cutoff_frequency{frequency_t{100.0}};

        /// Voltage magnitude low-pass filter cutoff frequency [Hz]
        parameter<frequency_t> voltage_magnitude_lpf_cutoff_frequency{frequency_t{10.0}};

        /// Commutation detection threshold (start) [pu]
        parameter<voltage_pu_t> commutation_start_threshold{voltage_pu_t{0.1f}};

        /// Commutation detection threshold (end) [pu]
        parameter<voltage_pu_t> commutation_end_threshold{voltage_pu_t{0.02f}};

        /// Time constant for smoothing commutation notch duration [s]
        parameter<duration_t> commutation_notch_duration_filter_time_constant{duration_t{10e-3F}};

        /// ADC resolution [bits]
        parameter<int> voltage_adc_bits{16};

        /// ADC full-scale voltage [V]
        parameter<real_t> voltage_adc_full_scale{real_t{10.0}};

        /// ADC resolution [bits]
        parameter<int> current_adc_bits{16};

        /// ADC full-scale voltage [V]
        parameter<real_t> current_adc_full_scale{real_t{10.0}};

        /// Voltage transducer gain [V_out/V_primary]
        parameter<real_t> voltage_transducer_gain{real_t{3.5 / 100.0}};

        /// Current transducer gain [V_out/A_primary]
        parameter<real_t> current_transducer_gain{real_t{1.0}};

        /// Potential transformer ratio [Primary/Secondary]
        parameter<real_t> pt_ratio{real_t{1.0}};

        /// Cutoff frequency of the hardware low-pass filter for voltage measurements [Hz]
        parameter<frequency_t> voltage_compensation_lpf_cutoff_frequency_1{frequency_t{0.0}};

        /// Cutoff frequency of the second hardware low-pass filter stage [Hz]
        parameter<frequency_t> voltage_compensation_lpf_cutoff_frequency_2{frequency_t{0.0}};

        /// Number of series-connected hardware low-pass filters to compensate
        parameter<int> voltage_compensation_num_filters{1};

        /// Enable phase-shift compensation for the hardware filter
        parameter<bool> enable_voltage_compensation{false};

        /// Enable commutation notch masking
        parameter<bool> enable_commutation_masking{true};

        /// Rotation angle for voltage measurements
        parameter<angle_wrapped> voltage_measurement_rotation{angle_wrapped{0.0f}};

        /// Current transformer ratio [Primary/Secondary]
        parameter<real_t> ct_ratio{real_t{1.0}};

        parameter<bool> clear_adc_peak_values{false};
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "measurements";
        // frequency_t parameters - convert to/from Hz
        handler.register_parameter(component_name, "voltage_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "current_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->current_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "voltage_magnitude_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_magnitude_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "voltage_compensation_lpf_cutoff_frequency_1",
            yeet::utils::make_units_accessor(_model_parameters->voltage_compensation_lpf_cutoff_frequency_1));
        handler.register_parameter(component_name, "voltage_compensation_lpf_cutoff_frequency_2",
            yeet::utils::make_units_accessor(_model_parameters->voltage_compensation_lpf_cutoff_frequency_2));
        handler.register_parameter(component_name, "voltage_compensation_num_filters",
            yeet::utils::make_units_accessor(_model_parameters->voltage_compensation_num_filters));
        handler.register_parameter(component_name, "enable_voltage_compensation",
            yeet::utils::make_bool_accessor(_model_parameters->enable_voltage_compensation));
        handler.register_parameter(component_name, "enable_commutation_masking",
            yeet::utils::make_bool_accessor(_model_parameters->enable_commutation_masking));
        handler.register_parameter(component_name, "voltage_measurement_rotation",
            yeet::utils::make_degrees_angle_wrapped_accessor(_model_parameters->voltage_measurement_rotation));

        // voltage_pu_t parameters - convert to/from per-unit
        handler.register_parameter(component_name, "commutation_start_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_start_threshold));
        handler.register_parameter(component_name, "commutation_end_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_end_threshold));
        handler.register_parameter(component_name, "commutation_notch_duration_filter_time_constant",
            yeet::utils::make_units_accessor(_model_parameters->commutation_notch_duration_filter_time_constant));

        // int and real_t parameters - use make_units_accessor
        handler.register_parameter(
            component_name, "voltage_adc_bits", yeet::utils::make_units_accessor(_model_parameters->voltage_adc_bits));
        handler.register_parameter(component_name, "voltage_adc_full_scale",
            yeet::utils::make_units_accessor(_model_parameters->voltage_adc_full_scale));
        handler.register_parameter(
            component_name, "current_adc_bits", yeet::utils::make_units_accessor(_model_parameters->current_adc_bits));
        handler.register_parameter(component_name, "current_adc_full_scale",
            yeet::utils::make_units_accessor(_model_parameters->current_adc_full_scale));
        handler.register_parameter(component_name, "voltage_transducer_gain",
            yeet::utils::make_units_accessor(_model_parameters->voltage_transducer_gain));
        handler.register_parameter(component_name, "current_transducer_gain",
            yeet::utils::make_units_accessor(_model_parameters->current_transducer_gain));
        handler.register_parameter(
            component_name, "pt_ratio", yeet::utils::make_units_accessor(_model_parameters->pt_ratio));
        handler.register_parameter(
            component_name, "ct_ratio", yeet::utils::make_units_accessor(_model_parameters->ct_ratio));
        handler.register_parameter(component_name, "clear_adc_peak_values",
            yeet::utils::make_bool_accessor(_model_parameters->clear_adc_peak_values));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
