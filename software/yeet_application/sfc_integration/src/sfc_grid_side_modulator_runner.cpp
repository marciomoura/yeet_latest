#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_modulator_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_modulator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
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
    const bool undervoltage_request_extinguish_dc_current = _inputs.undervoltage_request_extinguish_dc_current.read();
    const angle_t grid_reference_angle = _inputs.grid_reference_angle.read();

    // OR all shutdown sources: state machine, supervisor, protection system, and undervoltage logic
    const bool request_extinguish_current = _inputs.request_extinguish_dc_current.read() || shutdown_request ||
                                            protection_trip || undervoltage_request_extinguish_dc_current;

    // Update the modulator logic - enable controlled by supervisor via release_pulses
    _sfc_grid_side_modulator.update(enable, operating_mode, grid_reference_angle, request_force_commutation,
        current_is_zero, firing_angle_from_main_pi, request_extinguish_current);

    // Write outputs to the datamodel
    if (!request_extinguish_current && _parameters.force_open_loop_firing_angle.read()) {
        // Force open-loop firing angle for testing/commissioning
        _outputs.firing_angle = _parameters.open_loop_firing_angle_value.read();
        _outputs.force_commutation_acknowledged = false;
        _outputs.freeze_integrator = false;
        _outputs.modulator_state = grid_side_modulator_state_t::running;
        _outputs.release_pulses = true;
    }
    else {
        _outputs.firing_angle = _sfc_grid_side_modulator.get_firing_angle_rad();
        _outputs.force_commutation_acknowledged = _sfc_grid_side_modulator.get_force_commutation_acknowledged();
        _outputs.freeze_integrator = _sfc_grid_side_modulator.get_is_requesting_main_pi_reset();
        _outputs.modulator_state = _sfc_grid_side_modulator.get_current_state();
        _outputs.release_pulses = _sfc_grid_side_modulator.get_pulse_release();
        _outputs.firing_pulses = _sfc_grid_side_modulator.get_firing_pulses();
        _outputs.any_thyristor_fired = _sfc_grid_side_modulator.get_any_thyristor_fired();
        _outputs.virtual_reference_angle = _sfc_grid_side_modulator.get_virtual_reference_angle();
        _outputs.virtual_reference_active = _sfc_grid_side_modulator.is_virtual_reference_active();
        _outputs.individual_override_active = _sfc_grid_side_modulator.is_individual_pulse_override_active();
    }
}

void sfc_grid_side_modulator_runner::stop() { _sfc_grid_side_modulator.reset(); }

void sfc_grid_side_modulator_runner::set_parameters()
{
    // Configure pulse generator basics
    _sfc_grid_side_modulator.configure_pulse_width(_parameters.pulse_width.read());
    _sfc_grid_side_modulator.configure_reference_type(_parameters.reference_type.read());

    _sfc_grid_side_modulator.configure_force_commutation_angle(_parameters.force_commutation_angle.read());
    _sfc_grid_side_modulator.configure_commutation_timeout(_parameters.commutation_timeout.read());

    // Configure virtual reference
    _sfc_grid_side_modulator.configure_virtual_reference(_parameters.virtual_reference_frequency.read());

    // Configure pulse mask if override is enabled
    if (_parameters.enable_individual_pulse_override.read()) {
        _sfc_grid_side_modulator.configure_pulse_mask(_parameters.override_p1.read(), _parameters.override_p2.read(),
            _parameters.override_p3.read(), _parameters.override_p4.read(), _parameters.override_p5.read(),
            _parameters.override_p6.read());
    }
    else {
        // Default to all enabled
        _sfc_grid_side_modulator.configure_pulse_mask(true, true, true, true, true, true);
    }
}

void sfc_grid_side_modulator_runner::set_parameter_callback()
{
    _parameters.pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.reference_type.register_callback([&]() { set_parameters(); });
    _parameters.force_commutation_angle.register_callback([&]() { set_parameters(); });
    _parameters.commutation_timeout.register_callback([&]() { set_parameters(); });
    _parameters.virtual_reference_frequency.register_callback([&]() { set_parameters(); });
    _parameters.enable_virtual_reference.register_callback([&]() { set_parameters(); });
    _parameters.enable_individual_pulse_override.register_callback([&]() { set_parameters(); });
    _parameters.override_p1.register_callback([&]() { set_parameters(); });
    _parameters.override_p2.register_callback([&]() { set_parameters(); });
    _parameters.override_p3.register_callback([&]() { set_parameters(); });
    _parameters.override_p4.register_callback([&]() { set_parameters(); });
    _parameters.override_p5.register_callback([&]() { set_parameters(); });
    _parameters.override_p6.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
