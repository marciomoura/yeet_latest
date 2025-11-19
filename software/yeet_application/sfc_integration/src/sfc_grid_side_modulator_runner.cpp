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
    const bool enable = _inputs.release_pulses.read();
    const bool request_force_commutation = _inputs.request_force_commutation.read();
    const bool current_is_zero = _inputs.current_is_zero.read();
    const angle_t firing_angle_from_main_pi = _inputs.firing_angle.read();
    const sfc_operating_mode_t operating_mode = _inputs.operating_mode.read();
    const bool shutdown_request = _inputs.shutdown_request.read();
    const bool protection_trip = _inputs.protection_trip.read();

    // OR all shutdown sources: state machine, supervisor, and protection system
    const bool request_extinguish_current =
        _inputs.request_extinguish_dc_current.read() || shutdown_request || protection_trip;

    // Update the modulator logic - enable controlled by supervisor via release_pulses
    _sfc_grid_side_modulator.update(enable, operating_mode, request_force_commutation, current_is_zero,
        firing_angle_from_main_pi, request_extinguish_current);

    // Write outputs to the datamodel
    _outputs.firing_angle = _sfc_grid_side_modulator.get_firing_angle_rad();
    _outputs.force_commutation_acknowledged = _sfc_grid_side_modulator.get_force_commutation_acknowledged();
    _outputs.freeze_integrator = _sfc_grid_side_modulator.get_is_requesting_main_pi_reset();
    _outputs.modulator_state = _sfc_grid_side_modulator.get_current_state();
    _outputs.release_pulses = _sfc_grid_side_modulator.get_pulse_release();
}

void sfc_grid_side_modulator_runner::stop() { _sfc_grid_side_modulator.reset(); }

void sfc_grid_side_modulator_runner::set_parameters()
{
    // Note: force_commutation_angle and commutation_timeout parameters exist in datamodel
    // but are not yet configurable in the core sfc_grid_side_modulator component.
    // These are currently hardcoded in the core implementation:
    // - force_commutation_angle: 150° (hardcoded)
    // - commutation_timeout: Not yet implemented
    // Future enhancement: Add configure methods to the core component.
}

void sfc_grid_side_modulator_runner::set_parameter_callback()
{
    _parameters.force_commutation_angle.register_callback([&]() { set_parameters(); });
    _parameters.commutation_timeout.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
