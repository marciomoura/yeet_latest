#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"

#include <cmath>

namespace yeet::application::sfc::integration {

void sfc_machine_side_modulator_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_modulator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_machine_side_modulator_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    _sfc_machine_side_modulator.reset();
}

void sfc_machine_side_modulator_runner::run()
{
    // Read inputs from the datamodel
    const bool enable = _inputs.release_pulses.read();
    const sfc_operating_mode_t operating_mode = _inputs.operating_mode.read();
    const bool force_commutation_acknowledged = _inputs.force_commutation_acknowledged.read();
    const angle_t estimated_rotor_angle = _inputs.machine_reference_angle.read().get_radians();
    const angle_t virtual_rotor_angle = _inputs.virtual_rotor_angle.read().get_radians();
    const reference_frame_source_t frame_source = _inputs.reference_frame_source.read();
    const angle_t firing_angle_from_lut = _inputs.firing_angle.read();

    // Select rotor angle based on reference frame source from startup sequencer
    const angle_t selected_rotor_angle =
        (frame_source == reference_frame_source_t::VIRTUAL) ? virtual_rotor_angle : estimated_rotor_angle;

    // Update the modulator - enable controlled via release_pulses input
    _sfc_machine_side_modulator.update(
        enable, operating_mode, force_commutation_acknowledged, selected_rotor_angle, firing_angle_from_lut);

    // Write outputs to the datamodel
    _outputs.firing_angle = _sfc_machine_side_modulator.get_firing_angle_rad();
    _outputs.selected_firing_angle_reference = angle_wrapped(selected_rotor_angle);
    _outputs.release_pulses = _sfc_machine_side_modulator.get_release_pulses();
    _outputs.request_force_commutation = _sfc_machine_side_modulator.get_request_force_commutation();
    _outputs.commutation_state = _sfc_machine_side_modulator.get_commutation_state();
    _outputs.firing_pulses = _sfc_machine_side_modulator.get_firing_pulses();
    _outputs.any_thyristor_fired = _sfc_machine_side_modulator.get_any_thyristor_fired();
    _outputs.virtual_reference_angle = _sfc_machine_side_modulator.get_virtual_reference_angle();
    _outputs.virtual_reference_active = _sfc_machine_side_modulator.is_virtual_reference_active();
    _outputs.individual_override_active = _sfc_machine_side_modulator.is_individual_pulse_override_active();

    // Get current rotor sector from modulator (0-5), convert to 1-6 for diagnostics
    const int sector_0_indexed = _sfc_machine_side_modulator.get_current_rotor_sector();
    _outputs.current_rotor_sector = (sector_0_indexed == -1) ? 0 : static_cast<uint8_t>(sector_0_indexed + 1);
}

void sfc_machine_side_modulator_runner::stop() { _sfc_machine_side_modulator.reset(); }

void sfc_machine_side_modulator_runner::set_parameters()
{
    // Configure pulse generator basics
    _sfc_machine_side_modulator.configure_pulse_width(_parameters.pulse_width.read());
    _sfc_machine_side_modulator.configure_reference_type(_parameters.reference_type.read());

    // Configure forced commutation angle
    _sfc_machine_side_modulator.configure_forced_commutation_firing_angle(
        _parameters.forced_commutation_firing_angle.read());

    // Configure virtual reference
    _sfc_machine_side_modulator.configure_virtual_reference(_parameters.virtual_reference_frequency.read());

    // Configure pulse mask if override is enabled
    if (_parameters.enable_individual_pulse_override.read()) {
        _sfc_machine_side_modulator.configure_pulse_mask(_parameters.override_p1.read(), _parameters.override_p2.read(),
            _parameters.override_p3.read(), _parameters.override_p4.read(), _parameters.override_p5.read(),
            _parameters.override_p6.read());
    }
    else {
        // Default to all enabled
        _sfc_machine_side_modulator.configure_pulse_mask(true, true, true, true, true, true);
    }
}

void sfc_machine_side_modulator_runner::set_parameter_callback()
{
    _parameters.pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.reference_type.register_callback([&]() { set_parameters(); });
    _parameters.forced_commutation_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.virtual_reference_frequency.register_callback([&]() { set_parameters(); });
    _parameters.enable_virtual_reference.register_callback([&]() { set_parameters(); });
    _parameters.enable_individual_pulse_override.register_callback([&]() { set_parameters(); });
    _parameters.enable.register_callback([&]() { _sfc_machine_side_modulator.reset(); });
    _parameters.override_p1.register_callback([&]() { set_parameters(); });
    _parameters.override_p2.register_callback([&]() { set_parameters(); });
    _parameters.override_p3.register_callback([&]() { set_parameters(); });
    _parameters.override_p4.register_callback([&]() { set_parameters(); });
    _parameters.override_p5.register_callback([&]() { set_parameters(); });
    _parameters.override_p6.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
