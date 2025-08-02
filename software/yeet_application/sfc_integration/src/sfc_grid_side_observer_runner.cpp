#include "sfc_integration/sfc_grid_side_observer_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_observer_runner::initialize() {
    set_parameters();
    set_parameter_callback();
    _sfc_grid_side_observer.reset();
}

void sfc_grid_side_observer_runner::run() {
    _sfc_grid_side_observer.update(_inputs.grid_voltage_pu, _inputs.grid_side_pulses);

    _outputs.estimated_angle = _sfc_grid_side_observer.get_estimated_angle();
    _outputs.estimated_frequency = _sfc_grid_side_observer.get_estimated_frequency();
    _outputs.estimated_frequency_pu = _sfc_grid_side_observer.get_estimated_frequency_pu();
    _outputs.estimated_voltage_magnitude = _sfc_grid_side_observer.get_estimated_voltage_magnitude();
    _outputs.filtered_voltage_alphabeta = _sfc_grid_side_observer.get_filtered_voltage_alphabeta();
    _outputs.is_commuting = _sfc_grid_side_observer.is_commuting();
    _outputs.is_locked = _sfc_grid_side_observer.is_locked();
}

void sfc_grid_side_observer_runner::stop() { _sfc_grid_side_observer.reset(); }

void sfc_grid_side_observer_runner::set_parameters() {
    _sfc_grid_side_observer.configure_pll(_parameters.pll_proportional_gain, _parameters.pll_integral_gain,
                                          _parameters.grid_nominal_frequency);
    _sfc_grid_side_observer.configure_commutation_failure_threshold(_parameters.commutation_detection_threshold);
}

void sfc_grid_side_observer_runner::set_parameter_callback() {
    _parameters.pll_proportional_gain.register_callback([&]() { set_parameters(); });
    _parameters.pll_integral_gain.register_callback([&]() { set_parameters(); });
    _parameters.grid_nominal_frequency.register_callback([&]() { set_parameters(); });
    _parameters.commutation_detection_threshold.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration