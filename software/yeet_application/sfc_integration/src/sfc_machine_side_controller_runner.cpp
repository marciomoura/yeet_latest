#include "sfc_integration/sfc_machine_side_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // Initialize the firing angle filter
    _firing_angle_filter.reset();
    _firing_angle_filter.configure(100.0f);

    // Force initial table calculation
    recalculate_lookup_table();

    // Initialize outputs to safe values
    _outputs.machine_side_firing_angle = angle_wrapped::from_degrees(150.0);
}

void sfc_machine_side_controller_runner::run()
{
    real_t const machine_side_firing_angle =
        _lookup_table.get_firing_angle_deg(_inputs.machine_speed.read().value() / 50.0, _inputs.dc_link_current.read());

    _outputs.machine_side_firing_angle = _firing_angle_filter.update(machine_side_firing_angle);
}

void sfc_machine_side_controller_runner::stop()
{
    _outputs.machine_side_firing_angle = angle_wrapped::from_degrees(150.0);  // Safe shutdown angle
}

void sfc_machine_side_controller_runner::set_parameters() {}

void sfc_machine_side_controller_runner::set_parameter_callback()
{
    _parameters.gamma_setpoint_deg.register_callback([&]() { recalculate_lookup_table(); });
    _parameters.machine_side_commutation_reactance_pu.register_callback([&]() { recalculate_lookup_table(); });
    _parameters.dc_current_axis_pu.register_callback([&]() { recalculate_lookup_table(); });
    _parameters.speed_axis_pu.register_callback([&]() { recalculate_lookup_table(); });
}

void sfc_machine_side_controller_runner::recalculate_lookup_table()
{
    // Get current parameters
    real_t gamma_deg = _parameters.gamma_setpoint_deg;
    std::array<real_t, 6> current_axis = _parameters.dc_current_axis_pu;
    std::array<real_t, 6> speed_axis = _parameters.speed_axis_pu;
    real_t commutation_reactance_pu = _parameters.machine_side_commutation_reactance_pu;

    // Recalculate the lookup table
    _lookup_table.recalculate_table(gamma_deg, current_axis, speed_axis, commutation_reactance_pu);
}

}  // namespace yeet::application::sfc::integration
