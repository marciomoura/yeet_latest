#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

// ---------------- Ideal runner (existing) ----------------
void sfc_grid_side_measurements_ideal_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_measurements_ideal_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_grid_side_measurements_ideal_runner::initialize()
{
    set_filter_parameters();
    set_commutation_parameters();
    set_hardware_filter_parameters();
    set_parameter_callback();
}

void sfc_grid_side_measurements_ideal_runner::run()
{
    _measurements.update(_per_unit_system->get_grid_bases(), _inputs.grid_voltage_abc, _inputs.grid_current_abc);

    abc<voltage_pu_t> const measurement_voltage =
        _measurements.get_voltage_pu().rotate(_parameters.voltage_measurement_rotation.read());

    _commutation_monitor.update(measurement_voltage, _inputs.grid_pulses);
    _outputs.is_commuting = _commutation_monitor.is_commuting();
    _outputs.is_masking = _commutation_mask.is_masking();
    _outputs.commutation_notch_duration_last = _commutation_monitor.get_notch_duration_last();
    _outputs.commutation_notch_duration_filtered = _commutation_monitor.get_notch_duration_filtered();

    abc<voltage_pu_t> const masked_voltage_abc = _parameters.enable_commutation_masking.read()
                                                     ? _commutation_mask.update(measurement_voltage, _inputs.grid_angle,
                                                           _inputs.grid_frequency, _outputs.is_commuting)
                                                     : measurement_voltage;

    // Compensate for hardware filter
    abc<voltage_pu_t> const processed_voltage_abc =
        _parameters.enable_voltage_compensation.read()
            ? _voltage_compensator.update(masked_voltage_abc, _inputs.grid_frequency)
            : masked_voltage_abc;

    abc<current_pu_t> const measurement_current =
        _measurements.get_current_pu().rotate(_parameters.current_measurement_rotation.read());

    _measurement_filters.update(processed_voltage_abc, measurement_current);

    _outputs.grid_frequency = to_pu(_inputs.grid_frequency.read(), _per_unit_system->get_grid_bases().base_frequency);
    _outputs.grid_voltage_alphabeta = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_alphabeta_filtered = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_abc = processed_voltage_abc;
    _outputs.grid_voltage_abc_raw = measurement_voltage;
    _outputs.grid_voltage_abc_filtered = _measurement_filters.get_filtered_voltages_pu();
    _outputs.grid_current_abc = measurement_current;
    _outputs.grid_voltage_magnitude = processed_voltage_abc.to_alphabeta().magnitude();
    _outputs.grid_voltage_magnitude_filtered = _measurement_filters.get_filtered_voltage_magnitude_pu();

    // Populate SI outputs for telemetry
    _outputs.grid_voltage_line =
        _parameters.enable_voltage_compensation.read()
            ? line_voltage<voltage_t>(_voltage_compensator.update(
                  _measurements.get_voltage_line().to_phase_balanced(), _inputs.grid_frequency))
            : _measurements.get_voltage_line();
    _outputs.grid_current = _measurements.get_current().rotate(_parameters.current_measurement_rotation.read());
}

void sfc_grid_side_measurements_ideal_runner::stop() {}

void sfc_grid_side_measurements_ideal_runner::set_filter_parameters()
{
    _measurement_filters.configure_voltage_filter(_parameters.voltage_lpf_cutoff_frequency);
    _measurement_filters.configure_current_filter(_parameters.current_lpf_cutoff_frequency);
    _measurement_filters.configure_voltage_magnitude_filter(_parameters.voltage_magnitude_lpf_cutoff_frequency);
}

void sfc_grid_side_measurements_ideal_runner::set_commutation_parameters()
{
    _commutation_monitor = sfc_commutation_monitor{
        sfc_commutation_monitor::Config{.commutation_start_threshold = _parameters.commutation_start_threshold,
            .commutation_end_threshold = _parameters.commutation_end_threshold},
        duration_t{sampling_time}};
    _commutation_monitor.configure_notch_duration_filter_time_constant(
        _parameters.commutation_notch_duration_filter_time_constant.read());
}

void sfc_grid_side_measurements_ideal_runner::set_parameter_callback()
{
    _parameters.voltage_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.current_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.voltage_magnitude_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.commutation_start_threshold.register_callback([&]() { set_commutation_parameters(); });
    _parameters.commutation_end_threshold.register_callback([&]() { set_commutation_parameters(); });
    _parameters.commutation_notch_duration_filter_time_constant.register_callback([&]() {
        _commutation_monitor.configure_notch_duration_filter_time_constant(
            _parameters.commutation_notch_duration_filter_time_constant.read());
    });
    _parameters.voltage_compensation_lpf_cutoff_frequency_1.register_callback(
        [&]() { set_hardware_filter_parameters(); });
    _parameters.voltage_compensation_lpf_cutoff_frequency_2.register_callback(
        [&]() { set_hardware_filter_parameters(); });
    _parameters.voltage_compensation_num_filters.register_callback([&]() { set_hardware_filter_parameters(); });
    _parameters.enable_voltage_compensation.register_callback([&]() { /* Nothing needed, read in run() */ });
    _parameters.enable_commutation_masking.register_callback([&]() { /* Nothing needed, read in run() */ });
    _parameters.voltage_measurement_rotation.register_callback([&]() { /* Nothing needed, read in run() */ });
    _parameters.current_measurement_rotation.register_callback([&]() { /* Nothing needed, read in run() */ });
}

void sfc_grid_side_measurements_ideal_runner::set_hardware_filter_parameters()
{
    _voltage_compensator.configure(_parameters.voltage_compensation_lpf_cutoff_frequency_1.read(),
        _parameters.voltage_compensation_lpf_cutoff_frequency_2.read(),
        _parameters.voltage_compensation_num_filters.read());
}

// --------------- ADC-based runner (new) ------------------
void sfc_grid_side_measurements_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_measurements_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_grid_side_measurements_runner::initialize()
{
    set_filter_parameters();
    set_commutation_parameters();
    set_measurement_conversion_parameters();
    set_hardware_filter_parameters();
    // Configure transformer ratios if provided
    _measurements.configure_transformer_ratios(_parameters.pt_ratio.read(), _parameters.ct_ratio.read());
    // Configure transducer gains
    _measurements.configure_voltage_transducer_gain(_parameters.voltage_transducer_gain.read());
    _measurements.configure_current_transducer_gain(_parameters.current_transducer_gain.read());
    set_parameter_callback();
}

void sfc_grid_side_measurements_runner::run()
{
    // Convert ADC counts back to per-unit quantities
    _measurements.update(_per_unit_system->get_grid_bases(), _inputs.v_ab_counts, _inputs.v_bc_counts,
        _inputs.i_a_counts, _inputs.i_c_counts);

    if (_parameters.clear_adc_peak_values) {
        _measurements.clear_peaks();
    }

    abc<voltage_pu_t> const measurement_voltage =
        _measurements.get_voltage_pu().rotate(_parameters.voltage_measurement_rotation.read());

    // Reconstruct full abc voltages from line-to-line (assuming balanced measurement chain)
    // Using Vab and Vbc -> Va,Vb,Vc (set Va = 0 reference then add offsets to center) is already done inside
    // measurement filtering stage via masking. We proceed with commutation logic using recovered per-unit voltages.
    _commutation_monitor.update(measurement_voltage, _inputs.grid_pulses);
    _outputs.is_commuting = _commutation_monitor.is_commuting();
    _outputs.is_masking = _commutation_mask.is_masking();
    _outputs.commutation_notch_duration_last = _commutation_monitor.get_notch_duration_last();
    _outputs.commutation_notch_duration_filtered = _commutation_monitor.get_notch_duration_filtered();

    abc<voltage_pu_t> const masked_voltage_abc = _parameters.enable_commutation_masking.read()
                                                     ? _commutation_mask.update(measurement_voltage, _inputs.grid_angle,
                                                           _inputs.grid_frequency, _outputs.is_commuting)
                                                     : measurement_voltage;

    // Compensate for hardware filter
    abc<voltage_pu_t> const processed_voltage_abc =
        _parameters.enable_voltage_compensation.read()
            ? _voltage_compensator.update(masked_voltage_abc, _inputs.grid_frequency)
            : masked_voltage_abc;

    abc<current_pu_t> const measurement_current =
        _measurements.get_current_pu().rotate(_parameters.current_measurement_rotation.read());

    _measurement_filters.update(processed_voltage_abc, measurement_current);

    _outputs.grid_frequency = to_pu(_inputs.grid_frequency.read(), _per_unit_system->get_grid_bases().base_frequency);
    _outputs.grid_voltage_alphabeta = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_alphabeta_filtered = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_abc = processed_voltage_abc;
    _outputs.grid_voltage_abc_raw = measurement_voltage;
    _outputs.grid_voltage_abc_filtered = _measurement_filters.get_filtered_voltages_pu();
    _outputs.grid_current_abc = measurement_current;
    _outputs.grid_voltage_magnitude = processed_voltage_abc.to_alphabeta().magnitude();
    _outputs.grid_voltage_magnitude_filtered = _measurement_filters.get_filtered_voltage_magnitude_pu();

    // Populate SI outputs for telemetry
    _outputs.grid_voltage_line =
        _parameters.enable_voltage_compensation.read()
            ? line_voltage<voltage_t>(_voltage_compensator.update(
                  _measurements.get_voltage_line().to_phase_balanced(), _inputs.grid_frequency))
            : _measurements.get_voltage_line();
    _outputs.grid_current = _measurements.get_current().rotate(_parameters.current_measurement_rotation.read());

    // Populate intermediate measurement signals for debugging
    _outputs.voltage_ab_card = _measurements.get_voltage_ab_card();
    _outputs.current_a_card_voltage = _measurements.get_current_a_card_voltage();
    _outputs.voltage_ab_secondary = _measurements.get_voltage_ab_secondary();
    _outputs.current_a_secondary = _measurements.get_current_a_secondary();

    // Populate peak ADC counts
    _outputs.v_ab_counts_peak = static_cast<real_t>(_measurements.get_v_ab_counts_peak());
    _outputs.v_bc_counts_peak = static_cast<real_t>(_measurements.get_v_bc_counts_peak());
    _outputs.i_a_counts_peak = static_cast<real_t>(_measurements.get_i_a_counts_peak());
    _outputs.i_c_counts_peak = static_cast<real_t>(_measurements.get_i_c_counts_peak());
}

void sfc_grid_side_measurements_runner::stop() {}

void sfc_grid_side_measurements_runner::set_filter_parameters()
{
    _measurement_filters.configure_voltage_filter(_parameters.voltage_lpf_cutoff_frequency);
    _measurement_filters.configure_current_filter(_parameters.current_lpf_cutoff_frequency);
    _measurement_filters.configure_voltage_magnitude_filter(_parameters.voltage_magnitude_lpf_cutoff_frequency);
}

void sfc_grid_side_measurements_runner::set_commutation_parameters()
{
    _commutation_monitor = sfc_commutation_monitor{
        sfc_commutation_monitor::Config{.commutation_start_threshold = _parameters.commutation_start_threshold,
            .commutation_end_threshold = _parameters.commutation_end_threshold},
        duration_t{sampling_time}};
    _commutation_monitor.configure_notch_duration_filter_time_constant(
        _parameters.commutation_notch_duration_filter_time_constant.read());
}

void sfc_grid_side_measurements_runner::set_parameter_callback()
{
    _parameters.clear_adc_peak_values.register_callback([&]() {
        if (_parameters.clear_adc_peak_values) {
            _measurements.clear_peaks();
        }
    });

    _parameters.voltage_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.current_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.voltage_magnitude_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.commutation_start_threshold.register_callback([&]() { set_commutation_parameters(); });
    _parameters.commutation_end_threshold.register_callback([&]() { set_commutation_parameters(); });
    _parameters.commutation_notch_duration_filter_time_constant.register_callback([&]() {
        _commutation_monitor.configure_notch_duration_filter_time_constant(
            _parameters.commutation_notch_duration_filter_time_constant.read());
    });
    // ADC conversion parameter callbacks
    _parameters.voltage_adc_bits.register_callback([&]() { set_measurement_conversion_parameters(); });
    _parameters.voltage_adc_full_scale.register_callback([&]() { set_measurement_conversion_parameters(); });
    _parameters.current_adc_bits.register_callback([&]() { set_measurement_conversion_parameters(); });
    _parameters.current_adc_full_scale.register_callback([&]() { set_measurement_conversion_parameters(); });
    _parameters.pt_ratio.register_callback([&]() {
        _measurements.configure_transformer_ratios(_parameters.pt_ratio.read(), _parameters.ct_ratio.read());
    });
    _parameters.ct_ratio.register_callback([&]() {
        _measurements.configure_transformer_ratios(_parameters.pt_ratio.read(), _parameters.ct_ratio.read());
    });
    _parameters.voltage_transducer_gain.register_callback(
        [&]() { _measurements.configure_voltage_transducer_gain(_parameters.voltage_transducer_gain.read()); });
    _parameters.current_transducer_gain.register_callback(
        [&]() { _measurements.configure_current_transducer_gain(_parameters.current_transducer_gain.read()); });
    _parameters.voltage_compensation_lpf_cutoff_frequency_1.register_callback(
        [&]() { set_hardware_filter_parameters(); });
    _parameters.voltage_compensation_lpf_cutoff_frequency_2.register_callback(
        [&]() { set_hardware_filter_parameters(); });
    _parameters.voltage_compensation_num_filters.register_callback([&]() { set_hardware_filter_parameters(); });
    _parameters.enable_voltage_compensation.register_callback([&]() { /* Nothing needed, read in run() */ });
    _parameters.enable_commutation_masking.register_callback([&]() { /* Nothing needed, read in run() */ });
    _parameters.voltage_measurement_rotation.register_callback([&]() { /* Nothing needed, read in run() */ });
    _parameters.current_measurement_rotation.register_callback([&]() { /* Nothing needed, read in run() */ });
}

void sfc_grid_side_measurements_runner::set_measurement_conversion_parameters()
{
    _measurements.configure_voltage_conversion(
        _parameters.voltage_adc_bits.read(), _parameters.voltage_adc_full_scale.read());
    _measurements.configure_current_conversion(
        _parameters.current_adc_bits.read(), _parameters.current_adc_full_scale.read());
}

void sfc_grid_side_measurements_runner::set_hardware_filter_parameters()
{
    _voltage_compensator.configure(_parameters.voltage_compensation_lpf_cutoff_frequency_1.read(),
        _parameters.voltage_compensation_lpf_cutoff_frequency_2.read(),
        _parameters.voltage_compensation_num_filters.read());
}

}  // namespace yeet::application::sfc::integration