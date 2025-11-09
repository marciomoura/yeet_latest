#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"

#include <cmath>

namespace yeet::application::sfc::integration {

void sfc_machine_side_modulator_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_modulator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
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
    const sfc_operating_mode_t operating_mode = _inputs.operating_mode.read();
    const bool force_commutation_acknowledged = _inputs.force_commutation_acknowledged.read();
    const angle_t estimated_rotor_angle = _inputs.machine_reference_angle.read().get_radians();
    const angle_t virtual_rotor_angle = _inputs.virtual_rotor_angle.read().get_radians();
    const reference_frame_source_t frame_source = _inputs.reference_frame_source.read();
    const angle_t firing_angle_from_lut = _inputs.firing_angle.read();

    // Select rotor angle based on reference frame source from startup sequencer
    const angle_t selected_rotor_angle =
        (frame_source == reference_frame_source_t::VIRTUAL) ? virtual_rotor_angle : estimated_rotor_angle;

    // Update the modulator - pass true for enable (control is via parameters)
    _sfc_machine_side_modulator.update(_inputs.release_pulses, operating_mode, force_commutation_acknowledged,
        selected_rotor_angle, firing_angle_from_lut);

    // Write outputs to the datamodel
    _outputs.firing_angle = _sfc_machine_side_modulator.get_firing_angle_rad();
    _outputs.release_pulses = _sfc_machine_side_modulator.get_release_pulses();
    _outputs.request_force_commutation = _sfc_machine_side_modulator.get_request_force_commutation();
    _outputs.commutation_state = _sfc_machine_side_modulator.get_commutation_state();

    // Calculate current rotor sector (1-6) for diagnostics
    // Each sector is 60 degrees (π/3 radians) wide
    const real_t rotor_degrees = selected_rotor_angle.value() * 180.0 / 3.14159265359;
    const real_t normalized_degrees = std::fmod(rotor_degrees, 360.0);
    const uint8_t sector = static_cast<uint8_t>(normalized_degrees / 60.0) + 1;
    _outputs.current_rotor_sector = (sector > 6) ? 1 : sector;
}

void sfc_machine_side_modulator_runner::stop() { _sfc_machine_side_modulator.reset(); }

void sfc_machine_side_modulator_runner::set_parameters()
{
    // Configure pulse mode firing angle
    // const angle_t pulse_angle = _parameters.forced_commutation_firing_angle.read();
    // _sfc_machine_side_modulator.configure_forced_commutation_firing_angle_rad(pulse_angle);
}

void sfc_machine_side_modulator_runner::set_parameter_callback()
{
    _parameters.forced_commutation_firing_angle.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
