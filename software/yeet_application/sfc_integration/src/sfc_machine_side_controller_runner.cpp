#include "sfc_integration/sfc_machine_side_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_controller_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_machine_side_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    recalculate_lookup_table();

    _firing_angle_filter.configure(_firing_angle_filter.calculate_cutoff_frequency(ms(10.0)));

    _lookup_table.reset();
    _outputs.firing_angle = _lookup_table.get_firing_angle_rad();
}

void sfc_machine_side_controller_runner::run()
{
    _lookup_table.update(_inputs.dc_link_current.read());

    _outputs.firing_angle = _lookup_table.get_firing_angle_rad();
}

void sfc_machine_side_controller_runner::stop()
{
    _outputs.firing_angle = angle_wrapped::from_degrees(150.0);  // Safe shutdown angle
}

void sfc_machine_side_controller_runner::set_parameters() {}

void sfc_machine_side_controller_runner::set_parameter_callback()
{
    _parameters.dc_current_pu_lookup_table.register_callback([&]() { recalculate_lookup_table(); });
    _parameters.firing_angle_lookup_table.register_callback([&]() { recalculate_lookup_table(); });
}

void sfc_machine_side_controller_runner::recalculate_lookup_table()
{
    _lookup_table.configure_table(
        _parameters.dc_current_pu_lookup_table.read(), _parameters.firing_angle_lookup_table.read());
}

}  // namespace yeet::application::sfc::integration
