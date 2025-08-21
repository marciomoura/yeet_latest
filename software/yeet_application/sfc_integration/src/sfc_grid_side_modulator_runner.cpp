#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_modulator_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    _sfc_grid_side_modulator.reset();
}

void sfc_grid_side_modulator_runner::run()
{
    // Read inputs from the datamodel
    const bool request_force_commutation = _inputs.request_force_commutation.read();
    const bool dc_link_current_is_zero = _inputs.dc_link_current_is_zero.read();
    const angle_t firing_angle_from_main_pi = _inputs.firing_angle.read();
    const bool enable = _inputs.enable.read();

    // Update the modulator logic
    _sfc_grid_side_modulator.update(request_force_commutation, dc_link_current_is_zero, firing_angle_from_main_pi);

    // Write outputs to the datamodel
    _outputs.firing_angle = _sfc_grid_side_modulator.get_firing_angle_rad();
    _outputs.force_commutation_acknowledged = _sfc_grid_side_modulator.get_force_commutation_acknowledged();
    _outputs.is_requesting_main_pi_reset = _sfc_grid_side_modulator.get_is_requesting_main_pi_reset();
    _outputs.modulator_state = _sfc_grid_side_modulator.get_current_state();
}

void sfc_grid_side_modulator_runner::stop() { _sfc_grid_side_modulator.reset(); }

void sfc_grid_side_modulator_runner::set_parameters()
{
    // Configure the modulator with the force commutation angle
    const angle_t force_commutation_angle = _parameters.force_commutation_angle.read();
    _sfc_grid_side_modulator.configure_force_commutation_angle_rad(force_commutation_angle);

    // Note: commutation_timeout is not used by the core component yet, but is read here for completeness.
    const duration_t commutation_timeout = _parameters.commutation_timeout.read();
    // _sfc_grid_side_modulator.configure_commutation_timeout(commutation_timeout); // Future implementation
}

void sfc_grid_side_modulator_runner::set_parameter_callback()
{
    _parameters.force_commutation_angle.register_callback([&]() { set_parameters(); });
    _parameters.commutation_timeout.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
