#pragma once

#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
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
        /// Machine voltage abc [V]
        signal_input<abc<voltage_t>> machine_voltage_abc;

        /// Machine current abc [A]
        signal_input<abc<current_t>> machine_current_abc;

        /// Pulse signals for commutation detection
        signal_input<control::six_pulse_generator::pulses> machine_pulses;

        /// Electrical frequency estimate [Hz]
        signal_input<frequency_t> machine_frequency;

        /// Electrical angle reference [rad]
        signal_input<angle_wrapped> machine_reference_angle;
    };

    /**
     * @brief Output signals from the machine-side measurements component
     */
    struct outputs {
        /// Machine speed [pu]
        signal_output<speed_pu_t> machine_speed;

        /// Machine voltage in alpha-beta frame [pu]
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta;

        /// Machine voltage in alpha-beta frame, filtered [pu]
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta_filtered;

        /// Machine voltage in abc frame [pu]
        signal_output<abc<voltage_pu_t>> machine_voltage_abc;

        /// Machine voltage in abc frame, unmasked [pu]
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_unmasked;

        /// Machine voltage in abc frame, filtered [pu]
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_filtered;

        /// Machine voltage magnitude [pu]
        signal_output<voltage_pu_t> machine_voltage_magnitude;

        /// Machine voltage magnitude, filtered [pu]
        signal_output<voltage_pu_t> machine_voltage_magnitude_filtered;

        /// Machine current in abc frame [pu]
        signal_output<abc<current_pu_t>> machine_current_abc;

        /// Commutation activity indicator [bool]
        signal_output<bool> is_commuting;

        /// Last measured commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_last;

        /// Filtered commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_filtered;

        /// Line voltages (unmasked) [V]
        signal_output<line_voltage<voltage_t>> machine_voltage_line;
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

        /// Cutoff frequency of the hardware low-pass filter for voltage measurements [Hz]
        parameter<frequency_t> voltage_hardware_filter_cutoff_frequency{frequency_t{0.0}};

        /// Enable phase-shift compensation for the hardware filter
        parameter<bool> enable_voltage_compensation{false};

        /// Enable commutation notch masking
        parameter<bool> enable_commutation_masking{true};

        /// Rotation angle for voltage measurements [deg]
        parameter<angle_wrapped> voltage_measurement_rotation{angle_wrapped{0.0f}};

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

        /// Voltage transducer gain [V_out/V_primary]
        parameter<real_t> voltage_transducer_gain{real_t{3.5 / 100.0}};

        /// Potential transformer ratio [Primary/Secondary]
        parameter<real_t> pt_ratio{real_t{1.0}};
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

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "measurements_ideal";
        // frequency_t parameters - convert to/from Hz
        handler.register_parameter(component_name, "voltage_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "current_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->current_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "voltage_magnitude_lpf_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_magnitude_lpf_cutoff_frequency));
        handler.register_parameter(component_name, "voltage_hardware_filter_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_hardware_filter_cutoff_frequency));
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
        handler.register_parameter(component_name, "voltage_transducer_gain",
            yeet::utils::make_units_accessor(_model_parameters->voltage_transducer_gain));
        handler.register_parameter(
            component_name, "pt_ratio", yeet::utils::make_units_accessor(_model_parameters->pt_ratio));
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

        /// Last measured commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_last;

        /// Filtered commutation notch duration [s]
        signal_output<duration_t> commutation_notch_duration_filtered;

        /// Line voltages (unmasked) [V]
        signal_output<line_voltage<voltage_t>> machine_voltage_line;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {
        parameter<frequency_t> voltage_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> current_lpf_cutoff_frequency{frequency_t{100.0}};
        parameter<frequency_t> voltage_magnitude_lpf_cutoff_frequency{frequency_t{10.0}};

        /// Cutoff frequency of the hardware low-pass filter for voltage measurements [Hz]
        parameter<frequency_t> voltage_hardware_filter_cutoff_frequency{frequency_t{0.0}};

        /// Enable phase-shift compensation for the hardware filter
        parameter<bool> enable_voltage_compensation{false};

        /// Enable commutation notch masking
        parameter<bool> enable_commutation_masking{true};

        /// Rotation angle for voltage measurements [deg]
        parameter<angle_wrapped> voltage_measurement_rotation{angle_wrapped{0.0f}};

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
        /// Voltage transducer gain [V_out/V_primary]
        parameter<real_t> voltage_transducer_gain{real_t{3.5 / 100.0}};
        /// Potential transformer ratio [Primary/Secondary]
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
        handler.register_parameter(component_name, "voltage_hardware_filter_cutoff_frequency",
            yeet::utils::make_units_accessor(_model_parameters->voltage_hardware_filter_cutoff_frequency));
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
        handler.register_parameter(component_name, "voltage_transducer_gain",
            yeet::utils::make_units_accessor(_model_parameters->voltage_transducer_gain));
        handler.register_parameter(
            component_name, "pt_ratio", yeet::utils::make_units_accessor(_model_parameters->pt_ratio));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
