#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_measurements_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
}

void sfc_grid_side_measurements_runner::run()
{
    _measurements.update(
        _per_unit_system->get_grid_bases(), _inputs.grid_voltage_measurements, _inputs.grid_current_measurements);

    // Execute commutation monitor and mask
    _commutation_monitor.update(_measurements.get_voltage_pu(), _inputs.grid_pulses);
    _outputs.is_commuting = _commutation_monitor.is_commuting();

    abc<voltage_pu_t> const masked_voltage_abc = _commutation_mask.update(_measurements.get_voltage_pu(),
        _inputs.grid_side_angle, /*_inputs.grid_side_frequency*/ frequency_t{50.0}, _outputs.is_commuting);

    _measurement_filters.update(masked_voltage_abc, _measurements.get_current_pu());

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

void sfc_grid_side_measurements_runner::set_parameters()
{
    // No parameters to set - per-unit system is managed externally
}

void sfc_grid_side_measurements_runner::set_parameter_callback()
{
    // Register callbacks for non-per-unit parameters only
    // (No base value parameters to register callbacks for)
}

}  // namespace yeet::application::sfc::integration