#include "sfc/sfc_machine_side_modulator.hpp"

#include <cassert>
#include <cmath>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_machine_side_modulator::sfc_machine_side_modulator(duration_t sampling_time) : _sampling_time(sampling_time)
{
    // Validate sampling time (should be positive)
    assert(sampling_time.value() > 0.0 && "Sampling time must be positive");

    // Initialize pulse generator with single-pulse configuration (120 degrees)
    _pulse_generator.configure_pulse_mask(
        yeet::control::six_pulse_generator::pulse_mask{true, true, true, true, true, true});

    // Initialize to safe default state
    reset();
}

void sfc_machine_side_modulator::update(bool enable,
    sfc_operating_mode_t operating_mode,
    bool force_commutation_acknowledged,
    angle_t rotor_position_rad,
    angle_t firing_angle_rad) noexcept
{
    // 1. Logic for firing angle and state handling
    if (!enable) {
        _current_firing_angle_rad = get_safe_default_angle();
        _commutation_state = machine_side_commutation_state_t::running;
        _release_pulses = false;
        _request_force_commutation = false;
    }
    else {
        if (operating_mode == sfc_operating_mode_t::natural_commutation) {
            _current_firing_angle_rad = firing_angle_rad;
            _release_pulses = true;
            _request_force_commutation = false;
            _commutation_state = machine_side_commutation_state_t::running;
        }
        else {
            // Pulse mode (forced commutation)
            _current_firing_angle_rad = _forced_commutation_firing_angle_rad;

            if (_commutation_state == machine_side_commutation_state_t::waiting_for_ack) {
                if (force_commutation_acknowledged) {
                    _request_force_commutation = false;
                    _release_pulses = true;
                    _commutation_state = machine_side_commutation_state_t::running;
                }
                else {
                    _request_force_commutation = true;
                    _release_pulses = false;
                }
            }
            else {
                // Default state is running, will check for new firing event later
                _release_pulses = true;
                _request_force_commutation = false;
            }
        }
    }

    // 2. Single update of the pulse generator
    // We update with 'enable' to respect global enable, but masking for handshake
    // happens at the getter level.
    _pulse_generator.update(rotor_position_rad, _current_firing_angle_rad, enable);
    _any_thyristor_fired = _pulse_generator.get_any_thyristor_fired();

    // 3. Post-update logic to detect new firing events in pulse mode
    if (enable && operating_mode == sfc_operating_mode_t::forced_commutation &&
        _commutation_state == machine_side_commutation_state_t::running) {
        if (_any_thyristor_fired) {
            _request_force_commutation = true;
            _release_pulses = false;
            _commutation_state = machine_side_commutation_state_t::waiting_for_ack;
        }
    }
}

void sfc_machine_side_modulator::reset()
{
    // Reset to safe initial state
    _current_firing_angle_rad = get_safe_default_angle();

    // Reset forced commutation handshake state
    _commutation_state = machine_side_commutation_state_t::running;
    _release_pulses = true;
    _request_force_commutation = false;
    _any_thyristor_fired = false;

    // Reset pulse generator
    _pulse_generator.reset();
}

angle_t sfc_machine_side_modulator::get_firing_angle_rad() const noexcept { return _current_firing_angle_rad; }

bool sfc_machine_side_modulator::get_release_pulses() const noexcept { return _release_pulses; }

bool sfc_machine_side_modulator::get_request_force_commutation() const noexcept { return _request_force_commutation; }

machine_side_commutation_state_t sfc_machine_side_modulator::get_commutation_state() const noexcept
{
    return _commutation_state;
}

sfc_thyristor_pulses_t sfc_machine_side_modulator::get_firing_pulses() const noexcept
{
    if (!_release_pulses) {
        return {false, false, false, false, false, false};
    }

    return _pulse_generator.get_pulses();
}

bool sfc_machine_side_modulator::get_any_thyristor_fired() const noexcept { return _any_thyristor_fired; }

void sfc_machine_side_modulator::configure_forced_commutation_firing_angle(angle_t angle) noexcept
{
    _forced_commutation_firing_angle_rad = angle;
}

void sfc_machine_side_modulator::configure_pulse_width(angle_t width) noexcept
{
    _pulse_generator.configure_pulse_width(width);
}

void sfc_machine_side_modulator::configure_reference_type(control::six_pulse_generator::reference_type type) noexcept
{
    _pulse_generator.configure_reference_type(type);
}

void sfc_machine_side_modulator::configure_virtual_reference(frequency_t frequency) noexcept
{
    _pulse_generator.configure_virtual_reference(frequency, _sampling_time);
}

void sfc_machine_side_modulator::configure_pulse_mask(bool p1, bool p2, bool p3, bool p4, bool p5, bool p6) noexcept
{
    _pulse_generator.configure_pulse_mask({p1, p2, p3, p4, p5, p6});
    _pulse_generator.enable_pulse_mask(true);
}

int sfc_machine_side_modulator::get_current_rotor_sector() const noexcept
{
    return static_cast<int>(_pulse_generator.get_current_sector()) - 1;
}

angle_t sfc_machine_side_modulator::get_virtual_reference_angle() const noexcept
{
    return _pulse_generator.get_virtual_reference_angle();
}

bool sfc_machine_side_modulator::is_virtual_reference_active() const noexcept
{
    return _pulse_generator.is_virtual_reference_enabled();
}

bool sfc_machine_side_modulator::is_individual_pulse_override_active() const noexcept
{
    return _pulse_generator.is_pulse_mask_enabled();
}

angle_t sfc_machine_side_modulator::get_safe_default_angle() const noexcept
{
    // Return a safe default firing angle when system is disabled
    // Zero degrees is typically safe as it corresponds to natural commutation point
    return angle_t(0.0);
}

}  // namespace yeet::application::sfc