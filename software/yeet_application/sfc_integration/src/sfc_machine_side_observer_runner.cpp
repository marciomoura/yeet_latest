#include "sfc_integration/sfc_machine_side_observer_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_observer_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // _sfc_machine_side_observer.configure_pll(real_t kp, duration_t ti, frequency_t nominal_frequency)
    // _sfc_machine_side_observer.reset();
}

void sfc_machine_side_observer_runner::run()
{
    _sfc_machine_side_observer.update(_inputs.machine_voltage_pu);

    _outputs.estimated_voltage_magnitude = _sfc_machine_side_observer.get_estimated_voltage_magnitude();
    _outputs.estimated_frequency = _sfc_machine_side_observer.get_estimated_frequency();
    _outputs.estimated_speed = speed_pu_t{_sfc_machine_side_observer.get_estimated_frequency()};
    _outputs.estimated_frequency_pu = _sfc_machine_side_observer.get_estimated_frequency_pu();
    _outputs.estimated_position = _sfc_machine_side_observer.get_estimated_angle();
    _outputs.filtered_voltage_alphabeta = _sfc_machine_side_observer.get_filtered_voltage_alphabeta();
    _outputs.is_converged = _sfc_machine_side_observer.is_locked();
}

void sfc_machine_side_observer_runner::stop() { _sfc_machine_side_observer.reset(); }

void sfc_machine_side_observer_runner::set_parameters()
{
    // PLL gains are not directly exposed in the datamodel, using defaults.
    // _sfc_machine_side_observer.configure_pll(...)
}

void sfc_machine_side_observer_runner::set_parameter_callback()
{
    // No parameters to register callbacks for in this runner.
}

}  // namespace yeet::application::sfc::integration