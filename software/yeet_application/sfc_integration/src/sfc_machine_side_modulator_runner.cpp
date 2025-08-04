#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_modulator_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    _sfc_machine_side_modulator.reset();
}

void sfc_machine_side_modulator_runner::run()
{
    // Extract enable command - default to true if not connected
    bool enable = true;  // TODO: Connect from system controller

    // Convert operating mode from int to enum - default to load commutated mode
    sfc_operating_mode_t operating_mode =
        sfc::sfc_operating_mode_t::load_commutated;  // TODO: Connect from mode detector

    // Convert rotor position from angle_wrapped to angle_t
    angle_wrapped rotor_position_wrapped = _inputs.rotor_position_electrical;
    angle_t rotor_position_rad = angle_t(rotor_position_wrapped.get_radians());

    // Convert lookup table firing angle from angle_wrapped to angle_t - default to 90 degrees
    angle_t firing_angle_from_lut = angle_t(pi / 2.0);  // TODO: Connect from lookup table when implemented

    // Update the modulator
    _sfc_machine_side_modulator.update(enable, operating_mode, rotor_position_rad, firing_angle_from_lut);

    // Get outputs from modulator
    angle_t firing_angle_rad = _sfc_machine_side_modulator.get_firing_angle_rad();
    _outputs.firing_angle = angle_wrapped::from_radians(firing_angle_rad);

    // For now, generate simple firing pulses
    // In a full implementation, this would use a six_pulse_generator
    control::six_pulse_generator::pulses default_pulses{};
    _outputs.firing_pulses = default_pulses;

    // Set operating mode output
    _outputs.active_operating_mode = static_cast<int>(_sfc_machine_side_modulator.get_current_operating_mode());

    // Set pulse mode firing angle (same as main firing angle for now)
    _outputs.pulse_mode_firing_angle = angle_wrapped::from_radians(firing_angle_rad);

    // Calculate advance angle (difference from base angle)
    angle_wrapped base_angle = _parameters.pulse_mode_firing_angle;
    angle_wrapped current_angle = angle_wrapped::from_radians(firing_angle_rad);
    real_t advance_radians = current_angle.get_radians() - base_angle.get_radians();
    _outputs.advance_angle = angle_wrapped::from_radians(angle_t{advance_radians});

    // Set status outputs
    _outputs.is_enabled = enable;
    _outputs.has_commutation_failure = false;  // TODO: implement commutation failure detection

    // Calculate next firing position (60° electrical ahead)
    constexpr real_t sixty_degrees_rad = pi / 3.0;
    real_t next_position_rad = rotor_position_rad.value() + sixty_degrees_rad;
    _outputs.next_firing_position = angle_wrapped::from_radians(angle_t{next_position_rad});
}

void sfc_machine_side_modulator_runner::stop() { _sfc_machine_side_modulator.reset(); }

void sfc_machine_side_modulator_runner::set_parameters()
{
    // Configure pulse mode firing angle
    angle_wrapped pulse_angle_wrapped = _parameters.pulse_mode_firing_angle;
    angle_t pulse_angle = angle_t(pulse_angle_wrapped.get_radians());
    _sfc_machine_side_modulator.configure_pulse_mode_firing_angle_rad(pulse_angle);
}

void sfc_machine_side_modulator_runner::set_parameter_callback()
{
    _parameters.pulse_mode_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.advance_angle_current_threshold.register_callback([&]() { set_parameters(); });
    _parameters.max_advance_angle.register_callback([&]() { set_parameters(); });
    _parameters.firing_pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.commutation_timeout.register_callback([&]() { set_parameters(); });
    _parameters.min_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.max_firing_angle.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
