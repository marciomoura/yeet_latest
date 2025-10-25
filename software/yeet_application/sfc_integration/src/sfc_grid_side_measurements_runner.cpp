#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

// ---------------- Ideal runner (existing) ----------------
void sfc_grid_side_measurements_ideal_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_measurements_ideal_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_grid_side_measurements_ideal_runner::initialize()
{
    set_filter_parameters();
    set_commutation_parameters();
    set_parameter_callback();
}

void sfc_grid_side_measurements_ideal_runner::run()
{
    _measurements.update(
        _per_unit_system->get_grid_bases(), _inputs.grid_voltage_measurements, _inputs.grid_current_measurements);

    _commutation_monitor.update(_measurements.get_voltage_pu(), _inputs.grid_pulses);
    _outputs.is_commuting = _commutation_monitor.is_commuting();

    abc<voltage_pu_t> const masked_voltage_abc = _commutation_mask.update(
        _measurements.get_voltage_pu(), _inputs.grid_side_angle, _inputs.grid_side_frequency, _outputs.is_commuting);

    _measurement_filters.update(masked_voltage_abc, _measurements.get_current_pu());

    _outputs.grid_side_frequency =
        to_pu(_inputs.grid_side_frequency.read(), _per_unit_system->get_grid_bases().base_frequency);
    _outputs.grid_voltage_alphabeta = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_alphabeta_filtered = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_abc = masked_voltage_abc;
    _outputs.grid_voltage_abc_unmasked = _measurements.get_voltage_pu();
    _outputs.grid_voltage_abc_filtered = _measurement_filters.get_filtered_voltages_pu();
    _outputs.grid_current_abc = _measurements.get_current_pu();
    _outputs.grid_voltage_magnitude = masked_voltage_abc.to_alphabeta().magnitude();
    _outputs.grid_voltage_magnitude_filtered = _measurement_filters.get_filtered_voltage_magnitude_pu();
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
            .commutation_end_threshold = _parameters.commutation_end_threshold}};
}

void sfc_grid_side_measurements_ideal_runner::set_parameter_callback()
{
    _parameters.voltage_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.current_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.voltage_magnitude_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.commutation_start_threshold.register_callback([&]() { set_commutation_parameters(); });
    _parameters.commutation_end_threshold.register_callback([&]() { set_commutation_parameters(); });
}

// --------------- ADC-based runner (new) ------------------
void sfc_grid_side_measurements_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_measurements_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_grid_side_measurements_runner::initialize()
{
    set_filter_parameters();
    set_commutation_parameters();
    set_measurement_conversion_parameters();
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

    // Reconstruct full abc voltages from line-to-line (assuming balanced measurement chain)
    // Using Vab and Vbc -> Va,Vb,Vc (set Va = 0 reference then add offsets to center) is already done inside
    // measurement filtering stage via masking. We proceed with commutation logic using recovered per-unit voltages.
    _commutation_monitor.update(_measurements.get_voltage_pu(), _inputs.grid_pulses);
    _outputs.is_commuting = _commutation_monitor.is_commuting();

    abc<voltage_pu_t> const masked_voltage_abc = _commutation_mask.update(
        _measurements.get_voltage_pu(), _inputs.grid_side_angle, _inputs.grid_side_frequency, _outputs.is_commuting);

    _measurement_filters.update(masked_voltage_abc, _measurements.get_current_pu());

    _outputs.grid_side_frequency =
        to_pu(_inputs.grid_side_frequency.read(), _per_unit_system->get_grid_bases().base_frequency);
    _outputs.grid_voltage_alphabeta = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_alphabeta_filtered = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.grid_voltage_abc = masked_voltage_abc;
    _outputs.grid_voltage_abc_unmasked = _measurements.get_voltage_pu();
    _outputs.grid_voltage_abc_filtered = _measurement_filters.get_filtered_voltages_pu();
    _outputs.grid_current_abc = _measurements.get_current_pu();
    _outputs.grid_voltage_magnitude = masked_voltage_abc.to_alphabeta().magnitude();
    _outputs.grid_voltage_magnitude_filtered = _measurement_filters.get_filtered_voltage_magnitude_pu();
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
            .commutation_end_threshold = _parameters.commutation_end_threshold}};
}

void sfc_grid_side_measurements_runner::set_parameter_callback()
{
    _parameters.voltage_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.current_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.voltage_magnitude_lpf_cutoff_frequency.register_callback([&]() { set_filter_parameters(); });
    _parameters.commutation_start_threshold.register_callback([&]() { set_commutation_parameters(); });
    _parameters.commutation_end_threshold.register_callback([&]() { set_commutation_parameters(); });
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
}

void sfc_grid_side_measurements_runner::set_measurement_conversion_parameters()
{
    _measurements.configure_voltage_conversion(
        _parameters.voltage_adc_bits.read(), _parameters.voltage_adc_full_scale.read());
    _measurements.configure_current_conversion(
        _parameters.current_adc_bits.read(), _parameters.current_adc_full_scale.read());
}

}  // namespace yeet::application::sfc::integration