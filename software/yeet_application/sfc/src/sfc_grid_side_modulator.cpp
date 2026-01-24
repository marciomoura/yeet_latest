#include "sfc/sfc_grid_side_modulator.hpp"

#include <cassert>

namespace yeet::application::sfc {

sfc_grid_side_modulator::sfc_grid_side_modulator(duration_t sampling_time) : _sampling_time(sampling_time)
{
    // Initialize pulse generator with single-pulse configuration (120 degrees)
    _pulse_generator.configure_pulse_mask(
        yeet::control::six_pulse_generator::pulse_mask{true, true, true, true, true, true});

    // Initialize to safe default state
    reset();
}

void sfc_grid_side_modulator::update(bool enable,
    sfc_operating_mode_t operating_mode,
    angle_wrapped reference_angle,
    bool request_force_commutation,
    bool is_current_zero,
    angle_wrapped firing_angle,
    bool request_extinguish_current) noexcept
{
    // 0. Monotonicity Guard: prevent "backward" jumps from the PLL/Grid reference
    _reference_angle_filter.update(reference_angle);
    angle_wrapped monotonic_reference_angle = _reference_angle_filter.get_output();

    // PRIORITY 1: Handle extinguish current request (highest priority - shutdown/trip)
    if (request_extinguish_current) {
        _current_state = grid_side_modulator_state_t::extinguishing_current;
        _current_firing_angle_rad = _force_commutation_angle_rad;  // Apply inverting angle
        _force_commutation_acknowledged = false;
        _main_pi_reset_requested = true;  // Assert PI reset during extinguish
        _release_pulses = true;           // Keep pulses active for controlled shutdown

        // Update pulse generator with reference and angle
        _pulse_generator.update(monotonic_reference_angle, _current_firing_angle_rad, _release_pulses && enable);
        _any_thyristor_fired = _pulse_generator.get_any_thyristor_fired();
        return;
    }

    // If we were extinguishing and request is now cleared, reset to running state
    if (_current_state == grid_side_modulator_state_t::extinguishing_current) {
        _current_state = grid_side_modulator_state_t::running;
        _main_pi_reset_requested = false;
        _force_commutation_acknowledged = false;
    }

    // PRIORITY 2: Handle disable
    if (!enable) {
        _current_state = grid_side_modulator_state_t::running;
        _current_firing_angle_rad = get_safe_default_angle();
        _force_commutation_acknowledged = false;
        _main_pi_reset_requested = false;
        _release_pulses = false;

        // Keep pulse generator updated but disabled
        _pulse_generator.update(monotonic_reference_angle, _current_firing_angle_rad, false);
        _any_thyristor_fired = _pulse_generator.get_any_thyristor_fired();
        return;
    }

    // Grid-side always has its pulses released if the module is enabled
    _release_pulses = true;

    // Operating mode determines behavior
    switch (operating_mode) {
        case sfc_operating_mode_t::natural_commutation:
            // Natural commutation mode: ignore force commutation requests, always pass through PI angle
            _current_state = grid_side_modulator_state_t::running;
            _current_firing_angle_rad = firing_angle;
            _force_commutation_acknowledged = false;
            _main_pi_reset_requested = false;
            break;

        case sfc_operating_mode_t::forced_commutation: {
            // Forced commutation mode: respond to force commutation requests
            const auto state_update =
                compute_forced_commutation_state_update(request_force_commutation, is_current_zero, firing_angle);

            _current_state = state_update.new_state;
            _current_firing_angle_rad = state_update.new_firing_angle;
            _force_commutation_acknowledged = state_update.new_force_commutation_acknowledged;
            _main_pi_reset_requested = state_update.new_main_pi_reset_requested || is_current_zero;
            break;
        }
    }

    // Update pulse generator with the selected firing angle
    _pulse_generator.update(monotonic_reference_angle, _current_firing_angle_rad, _release_pulses && enable);
    _any_thyristor_fired = _pulse_generator.get_any_thyristor_fired();
}
sfc_grid_side_modulator::forced_commutation_state_update
sfc_grid_side_modulator::compute_forced_commutation_state_update(
    bool request_force_commutation, bool is_current_zero, angle_wrapped firing_angle) const noexcept
{
    sfc_grid_side_modulator::forced_commutation_state_update state_update{};

    switch (_current_state) {
        case grid_side_modulator_state_t::running:
            // If a force commutation is requested, switch to forced commutation state
            // which should bring the dc-link current down
            if (request_force_commutation) {
                // Applying the forced commutation angle immediately
                state_update.new_state = grid_side_modulator_state_t::forced_commutation;
                state_update.new_firing_angle = _force_commutation_angle_rad;
                // Acknowledge immediately if DC link current is already zero
                state_update.new_force_commutation_acknowledged = false;
                state_update.new_main_pi_reset_requested = true;
            }
            else {
                state_update.new_state = grid_side_modulator_state_t::running;
                state_update.new_firing_angle = firing_angle;
                state_update.new_force_commutation_acknowledged = false;
                state_update.new_main_pi_reset_requested = false;
            }
            break;

        case grid_side_modulator_state_t::forced_commutation:
            // The condition to move back to running state is
            // the signal `is_current_zero`
            if (is_current_zero) {
                // In case dc-link is zero, the machine-side will clear the force-commutation request.
                // Hence, the grid-side will move back to running state and will clear the acknowledgment flag.
                if (!request_force_commutation) {
                    // Commutation is complete, return to normal operation
                    state_update.new_state = grid_side_modulator_state_t::running;
                    state_update.new_firing_angle = firing_angle;
                    state_update.new_force_commutation_acknowledged = false;
                    state_update.new_main_pi_reset_requested = false;
                }
                else {
                    state_update.new_state =
                        grid_side_modulator_state_t::forced_commutation;           ///< Forced commutation state
                    state_update.new_firing_angle = _force_commutation_angle_rad;  ///< Force commutation angle
                    state_update.new_force_commutation_acknowledged = true;        ///< Force commutation acknowledged
                    state_update.new_main_pi_reset_requested = true;               ///< Main PI reset requested
                }
            }
            else {
                // Continue applying the forced commutation angle
                state_update.new_state = grid_side_modulator_state_t::forced_commutation;  ///< Forced commutation state
                state_update.new_firing_angle = _force_commutation_angle_rad;              ///< Force commutation angle
                state_update.new_force_commutation_acknowledged = false;  ///< Force commutation acknowledged
                state_update.new_main_pi_reset_requested = true;          ///< Main PI reset requested
            }
            break;

        default:
            assert(false && "Unhandled state in sfc_grid_side_modulator");
            // Return safe default to satisfy compiler
            state_update.new_state = grid_side_modulator_state_t::running;
            state_update.new_firing_angle = get_safe_default_angle();
            state_update.new_force_commutation_acknowledged = false;
            state_update.new_main_pi_reset_requested = false;
            break;
    }

    return state_update;
}

void sfc_grid_side_modulator::reset()
{
    _current_state = grid_side_modulator_state_t::running;
    _current_firing_angle_rad = get_safe_default_angle();
    _force_commutation_acknowledged = false;
    _main_pi_reset_requested = false;
    _release_pulses = true;
    _any_thyristor_fired = false;
    _reference_angle_filter.reset();

    // Reset pulse generator
    _pulse_generator.reset();
}

void sfc_grid_side_modulator::configure_virtual_reference(frequency_t frequency) noexcept
{
    _pulse_generator.configure_virtual_reference(frequency, _sampling_time);
    _pulse_generator.enable_virtual_reference(frequency.value() > 0.0);
}

void sfc_grid_side_modulator::configure_pulse_mask(bool p1, bool p2, bool p3, bool p4, bool p5, bool p6) noexcept
{
    _pulse_generator.configure_pulse_mask({p1, p2, p3, p4, p5, p6});
    _pulse_generator.enable_pulse_mask(true);
}

sfc_thyristor_pulses_t sfc_grid_side_modulator::get_firing_pulses() const noexcept
{
    if (!_release_pulses) {
        return sfc_thyristor_pulses_t{};
    }

    return _pulse_generator.get_pulses();
}

bool sfc_grid_side_modulator::get_any_thyristor_fired() const noexcept { return _any_thyristor_fired; }

void sfc_grid_side_modulator::configure_pulse_width(angle_t width) noexcept
{
    _pulse_generator.configure_pulse_width(width);
}

void sfc_grid_side_modulator::configure_reference_type(control::six_pulse_generator::reference_type type) noexcept
{
    _pulse_generator.configure_reference_type(type);
}

void sfc_grid_side_modulator::configure_force_commutation_angle(angle_t angle) noexcept
{
    _force_commutation_angle_rad = angle;
}

void sfc_grid_side_modulator::configure_commutation_timeout(duration_t timeout) noexcept
{
    _commutation_timeout = timeout;
}

angle_t sfc_grid_side_modulator::get_firing_angle_rad() const noexcept { return _current_firing_angle_rad; }

grid_side_modulator_state_t sfc_grid_side_modulator::get_current_state() const noexcept { return _current_state; }

bool sfc_grid_side_modulator::get_force_commutation_acknowledged() const noexcept
{
    return _force_commutation_acknowledged;
}

bool sfc_grid_side_modulator::get_is_requesting_main_pi_reset() const noexcept { return _main_pi_reset_requested; }

void sfc_grid_side_modulator::acknowledge_main_pi_reset() { _main_pi_reset_requested = false; }

angle_t sfc_grid_side_modulator::get_safe_default_angle() const noexcept
{
    // Return 0° (natural commutation point) as the safe default
    return angle_t(0.0f);
}

bool sfc_grid_side_modulator::get_pulse_release() const noexcept { return _release_pulses; }

bool sfc_grid_side_modulator::get_is_extinguishing_current() const noexcept
{
    return _current_state == grid_side_modulator_state_t::extinguishing_current;
}

angle_t sfc_grid_side_modulator::get_virtual_reference_angle() const noexcept
{
    return _pulse_generator.get_virtual_reference_angle();
}

angle_t sfc_grid_side_modulator::get_monotonic_reference_angle() const noexcept
{
    return _reference_angle_filter.get_output();
}

bool sfc_grid_side_modulator::is_virtual_reference_active() const noexcept
{
    return _pulse_generator.is_virtual_reference_enabled();
}

bool sfc_grid_side_modulator::is_individual_pulse_override_active() const noexcept
{
    return _pulse_generator.is_pulse_mask_enabled();
}

}  // namespace yeet::application::sfc
