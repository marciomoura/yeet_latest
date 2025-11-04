#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_modulator_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_modulator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

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
    const bool is_current_zero = _inputs.is_current_zero.read();
    const angle_t firing_angle_from_main_pi = _inputs.firing_angle.read();
    const sfc_operating_mode_t operating_mode = _inputs.operating_mode.read();
    const bool is_tripped = _inputs.is_tripped.read();

    // Update the modulator logic - pass true for enable (control is via parameters and is_tripped)
    _sfc_grid_side_modulator.update(true, operating_mode, request_force_commutation, is_current_zero,
        firing_angle_from_main_pi, _inputs.request_extinguish_dc_current.read());

    // Apply enable parameter override to release_pulses
    const bool modulator_release_pulses = _sfc_grid_side_modulator.get_pulse_release();
    const bool enable_override = _parameters.enable.read();
    const bool final_release_pulses = modulator_release_pulses && enable_override && !is_tripped;

    // Write outputs to the datamodel
    _outputs.firing_angle = _sfc_grid_side_modulator.get_firing_angle_rad();
    _outputs.force_commutation_acknowledged = _sfc_grid_side_modulator.get_force_commutation_acknowledged();
    _outputs.freeze_integrator = _sfc_grid_side_modulator.get_is_requesting_main_pi_reset();
    _outputs.modulator_state = _sfc_grid_side_modulator.get_current_state();
    _outputs.release_pulses = final_release_pulses;
}

void sfc_grid_side_modulator_runner::stop() { _sfc_grid_side_modulator.reset(); }

void sfc_grid_side_modulator_runner::set_parameters()
{
    // Configure the modulator with the force commutation angle
    // const angle_t force_commutation_angle = _parameters.force_commutation_angle.read();
    // _sfc_grid_side_modulator.configure_force_commutation_angle_rad(force_commutation_angle);

    // Note: commutation_timeout is not used by the core component yet, but is read here for completeness.
    // const duration_t commutation_timeout = _parameters.commutation_timeout.read();
    // _sfc_grid_side_modulator.configure_commutation_timeout(commutation_timeout); // Future implementation
}

void sfc_grid_side_modulator_runner::set_parameter_callback()
{
    _parameters.force_commutation_angle.register_callback([&]() { set_parameters(); });
    _parameters.commutation_timeout.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
