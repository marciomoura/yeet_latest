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
    const angle_t rotor_position_rad = _inputs.machine_side_angle.read();
    const angle_t firing_angle_from_lut = _inputs.firing_angle.read();

    // Update the modulator - pass true for enable (control is via parameters)
    _sfc_machine_side_modulator.update(
        true, operating_mode, force_commutation_acknowledged, rotor_position_rad, firing_angle_from_lut);

    // Apply enable parameter override to release_pulses
    const bool modulator_release_pulses = _sfc_machine_side_modulator.get_release_pulses();
    const bool enable_override = _parameters.enable.read();
    const bool final_release_pulses = modulator_release_pulses && enable_override;

    // Write outputs to the datamodel
    _outputs.firing_angle = _sfc_machine_side_modulator.get_firing_angle_rad();
    _outputs.release_pulses = final_release_pulses;
    _outputs.request_force_commutation = _sfc_machine_side_modulator.get_request_force_commutation();
    _outputs.commutation_state = _sfc_machine_side_modulator.get_commutation_state();

    // Calculate current rotor sector (1-6) for diagnostics
    // Each sector is 60 degrees (π/3 radians) wide
    const real_t rotor_degrees = rotor_position_rad.value() * 180.0 / 3.14159265359;
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
