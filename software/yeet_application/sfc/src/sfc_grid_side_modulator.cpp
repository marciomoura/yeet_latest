#include "sfc/sfc_grid_side_modulator.hpp"

#include <cassert>
#include <cmath>

namespace yeet::application::sfc {

sfc_grid_side_modulator::sfc_grid_side_modulator(duration_t sampling_time) : _pulse_state_timer(sampling_time) {
    // Validate sampling time
    assert(sampling_time.value() > 0.0 && "Sampling time must be positive");

    // Initialize to safe default state
    reset();
}

void sfc_grid_side_modulator::configure_pulse_mode_current_thresholds(current_pu_t zero_threshold,
                                                                      current_pu_t setpoint_reached_threshold) {
    assert(zero_threshold.value() >= 0.0 && "Zero threshold must be non-negative");
    assert(setpoint_reached_threshold.value() >= 0.0 && "Setpoint threshold must be non-negative");
    assert(setpoint_reached_threshold.value() >= zero_threshold.value() &&
           "Setpoint threshold must be >= zero threshold");

    _zero_threshold_pu = zero_threshold;
    _setpoint_reached_threshold_pu = setpoint_reached_threshold;
}

void sfc_grid_side_modulator::configure_force_commutation_angle_rad(angle_t angle) {
    _force_commutation_angle_rad = angle;
}

void sfc_grid_side_modulator::configure_pulse_mode_timing(duration_t min_pulse_duration_s,
                                                          duration_t max_pulse_duration_s,
                                                          duration_t commutation_timeout_s) {
    assert(min_pulse_duration_s.value() >= 0.0 && "Min pulse duration must be non-negative");
    assert(max_pulse_duration_s.value() >= min_pulse_duration_s.value() &&
           "Max pulse duration must be >= min pulse duration");
    assert(commutation_timeout_s.value() > 0.0 && "Commutation timeout must be positive");

    _min_pulse_duration_s = min_pulse_duration_s;
    _max_pulse_duration_s = max_pulse_duration_s;
    _commutation_timeout_s = commutation_timeout_s;
}

void sfc_grid_side_modulator::update(bool enable, sfc_operating_mode_t operating_mode, current_pu_t current_setpoint_pu,
                                     current_pu_t measured_dc_current_pu, angle_t firing_angle_from_main_controller,
                                     bool rotor_sector_change_request) noexcept {
    // Store system enable state and operating mode
    _system_enabled = enable;
    _previous_mode = _current_mode;
    _current_mode = operating_mode;

    // If system is disabled, output safe default angle
    if (!enable) {
        _current_firing_angle_rad = get_safe_default_angle();
        _pulse_state = pulse_mode_state_t::idle;
        _pulse_state_timer.reset();
        return;
    }

    // Check for mode transitions and handle PI reset request
    if (_previous_mode == sfc_operating_mode_t::pulse_mode && _current_mode == sfc_operating_mode_t::load_commutated) {
        // Transitioning from Pulse Mode to Load-Commutated Mode
        _main_pi_reset_requested = true;
        _pulse_state = pulse_mode_state_t::idle;
        _pulse_state_timer.reset();
    }

    // Select firing angle based on operating mode
    switch (_current_mode) {
        case sfc_operating_mode_t::load_commutated:
            // Pass through the firing angle from external current controller
            _current_firing_angle_rad = firing_angle_from_main_controller;
            _pulse_state = pulse_mode_state_t::idle;
            break;

        case sfc_operating_mode_t::pulse_mode:
            // Update pulse mode state machine and calculate firing angle
            update_pulse_mode_state_machine(current_setpoint_pu, measured_dc_current_pu, rotor_sector_change_request);
            _current_firing_angle_rad = calculate_pulse_mode_firing_angle(firing_angle_from_main_controller);
            break;
    }

    // Update the elapsed timer - it will only increment when enabled
    _pulse_state_timer.update(_system_enabled && _current_mode == sfc_operating_mode_t::pulse_mode);
}

void sfc_grid_side_modulator::reset() {
    // Reset to safe initial state
    _current_mode = sfc_operating_mode_t::pulse_mode;
    _previous_mode = sfc_operating_mode_t::pulse_mode;
    _pulse_state = pulse_mode_state_t::idle;
    _current_firing_angle_rad = get_safe_default_angle();
    _system_enabled = false;
    _main_pi_reset_requested = false;
    _pulse_state_timer.reset();
    _previous_setpoint_pu = current_pu_t(0.0);
}

angle_t sfc_grid_side_modulator::get_firing_angle_rad() const noexcept { return _current_firing_angle_rad; }

sfc_operating_mode_t sfc_grid_side_modulator::get_current_operating_mode() const noexcept { return _current_mode; }

pulse_mode_state_t sfc_grid_side_modulator::get_pulse_mode_state() const noexcept { return _pulse_state; }

bool sfc_grid_side_modulator::is_requesting_main_pi_reset() const noexcept { return _main_pi_reset_requested; }

void sfc_grid_side_modulator::acknowledge_main_pi_reset() { _main_pi_reset_requested = false; }

void sfc_grid_side_modulator::update_pulse_mode_state_machine(current_pu_t current_setpoint_pu,
                                                              current_pu_t measured_dc_current_pu,
                                                              bool rotor_sector_change_request) noexcept {
    // Store previous state for transition detection
    const pulse_mode_state_t previous_pulse_state = _pulse_state;

    switch (_pulse_state) {
        case pulse_mode_state_t::idle:
            // Wait for enable and non-zero setpoint
            if (_system_enabled && current_setpoint_pu.value() > _zero_threshold_pu.value()) {
                _pulse_state = pulse_mode_state_t::setup_pulse;
                _pulse_state_timer.reset();
            }
            break;

        case pulse_mode_state_t::setup_pulse:
            // Prepare for current pulse - transition immediately to pulse_on
            _pulse_state = pulse_mode_state_t::pulse_on;
            _pulse_state_timer.reset();
            break;

        case pulse_mode_state_t::pulse_on:
            // Build DC link current using external controller
            // Exit conditions:
            // 1. Rotor sector change request (normal completion)
            // 2. Maximum pulse duration exceeded (safety timeout)
            // 3. Setpoint reached and minimum time elapsed

            if (rotor_sector_change_request) {
                // Normal pulse completion - rotor moved to next sector
                _pulse_state = pulse_mode_state_t::force_commutation;
                _pulse_state_timer.reset();
            } else if (_pulse_state_timer.has_elapsed(_max_pulse_duration_s)) {
                // Safety timeout - force commutation
                _pulse_state = pulse_mode_state_t::force_commutation;
                _pulse_state_timer.reset();
            } else if (_pulse_state_timer.has_elapsed(_min_pulse_duration_s) &&
                       is_setpoint_reached(current_setpoint_pu, measured_dc_current_pu)) {
                // Setpoint reached and minimum time elapsed - continue until sector change
                // Stay in pulse_on state
            }
            break;

        case pulse_mode_state_t::force_commutation:
            // Force current to zero for commutation
            // Exit when current reaches zero or timeout

            if (is_current_zero(measured_dc_current_pu)) {
                // Commutation successful - return to idle for next pulse
                _pulse_state = pulse_mode_state_t::idle;
                _pulse_state_timer.reset();
            } else if (_pulse_state_timer.has_elapsed(_commutation_timeout_s)) {
                // Commutation timeout - return to idle (fault condition should be handled externally)
                _pulse_state = pulse_mode_state_t::idle;
                _pulse_state_timer.reset();
            }
            break;
    }

    // Reset timer on state transitions (if not already reset above)
    if (_pulse_state != previous_pulse_state && _pulse_state != pulse_mode_state_t::force_commutation &&
        _pulse_state != pulse_mode_state_t::pulse_on && _pulse_state != pulse_mode_state_t::idle) {
        _pulse_state_timer.reset();
    }
}

angle_t sfc_grid_side_modulator::calculate_pulse_mode_firing_angle(
    angle_t firing_angle_from_main_controller) const noexcept {
    switch (_pulse_state) {
        case pulse_mode_state_t::idle:
        case pulse_mode_state_t::setup_pulse:
            // Output safe angle when not actively pulsing
            return get_safe_default_angle();

        case pulse_mode_state_t::pulse_on:
            // Use external controller output to build current
            return firing_angle_from_main_controller;

        case pulse_mode_state_t::force_commutation:
            // Use fixed deep inverting angle to force current to zero
            return _force_commutation_angle_rad;

        default:
            return get_safe_default_angle();
    }
}

angle_t sfc_grid_side_modulator::get_safe_default_angle() const noexcept {
    // Return a safe default firing angle when system is disabled
    // Zero degrees (natural commutation point) is typically safe
    return angle_t(0.0);
}

bool sfc_grid_side_modulator::is_current_zero(current_pu_t current_pu) const noexcept {
    return std::abs(current_pu.value()) < _zero_threshold_pu.value();
}

bool sfc_grid_side_modulator::is_setpoint_reached(current_pu_t setpoint_pu, current_pu_t measured_pu) const noexcept {
    const real_t error = std::abs(setpoint_pu.value() - measured_pu.value());
    return error < _setpoint_reached_threshold_pu.value();
}

}  // namespace yeet::application::sfc