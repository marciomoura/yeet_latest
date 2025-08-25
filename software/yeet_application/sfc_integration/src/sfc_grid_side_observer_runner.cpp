#include "sfc_integration/sfc_grid_side_observer_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_observer_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    _sfc_grid_side_observer.reset(_parameters.initial_frequency);
}

void sfc_grid_side_observer_runner::run()
{
    _sfc_grid_side_observer.update(_inputs.grid_voltage_pu);

    _outputs.estimated_position = _sfc_grid_side_observer.get_estimated_angle();
    _outputs.estimated_frequency = _sfc_grid_side_observer.get_estimated_frequency();
    _outputs.estimated_frequency_pu = _sfc_grid_side_observer.get_estimated_frequency_pu();
    _outputs.estimated_voltage_magnitude = _sfc_grid_side_observer.get_estimated_voltage_magnitude();
    _outputs.filtered_voltage_alphabeta = _sfc_grid_side_observer.get_filtered_voltage_alphabeta();
}

void sfc_grid_side_observer_runner::stop() { _sfc_grid_side_observer.reset(); }

void sfc_grid_side_observer_runner::set_parameters()
{
    _sfc_grid_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
}

void sfc_grid_side_observer_runner::set_parameter_callback()
{
    _parameters.kp.register_callback([&]() { set_parameters(); });
    _parameters.ti.register_callback([&]() { set_parameters(); });
    _parameters.nominal_frequency.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration