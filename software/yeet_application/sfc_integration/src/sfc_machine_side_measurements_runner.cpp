#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_measurements_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
}

void sfc_machine_side_measurements_runner::run()
{
    _measurements.update(_per_unit_system->get_machine_bases(), _inputs.machine_voltage_measurements,
        _inputs.machine_current_measurements);

    // Execute commutation monitor and mask
    _commutation_monitor.update(_measurements.get_voltage_pu(), _inputs.machine_pulses);
    _outputs.is_commuting = _commutation_monitor.is_commuting();
    abc<voltage_pu_t> const masked_voltage_abc = _commutation_mask.update(_measurements.get_voltage_pu(),
        _inputs.machine_side_angle, _inputs.machine_side_frequency, _outputs.is_commuting);

    _measurement_filters.update(masked_voltage_abc, _measurements.get_current_pu());

    _outputs.machine_voltage_alphabeta = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.machine_voltage_alphabeta_filtered = _measurement_filters.get_filtered_voltage_alphabeta_pu();
    _outputs.machine_voltage_abc = masked_voltage_abc;
    _outputs.machine_voltage_abc_unmasked = _measurements.get_voltage_pu();
    _outputs.machine_voltage_abc_filtered = _measurement_filters.get_filtered_voltages_pu();
    _outputs.machine_current_abc = _measurements.get_current_pu();
    _outputs.machine_voltage_magnitude = masked_voltage_abc.to_alphabeta().magnitude();
    _outputs.machine_voltage_magnitude_filtered = _measurement_filters.get_filtered_voltage_magnitude_pu();
}

void sfc_machine_side_measurements_runner::stop() {}

void sfc_machine_side_measurements_runner::set_parameters()
{
    // Configure filtering parameters (non-per-unit related)
    // _measurement_filters.configure_voltage_filter(_parameters.voltage_lpf_cutoff);
    // _measurement_filters.configure_current_filter(_parameters.current_lpf_cutoff);
    // _measurement_filters.configure_voltage_magnitude_filter(_parameters.voltage_lpf_cutoff);
}

void sfc_machine_side_measurements_runner::set_parameter_callback()
{
    // Register callbacks for non-per-unit parameters only
    // _parameters.voltage_lpf_cutoff.register_callback([&]() { set_parameters(); });
    // _parameters.current_lpf_cutoff.register_callback([&]() { set_parameters(); });
    // _parameters.speed_lpf_cutoff.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration