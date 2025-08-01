#include "sfc/sfc_mode_detector.hpp"

#include <algorithm>
#include <cassert>

namespace yeet::application::sfc {

sfc_mode_detector::sfc_mode_detector(duration_t sampling_time) : _sampling_time(sampling_time) {
    // Validate sampling time
    assert(sampling_time.value() > 0.0 && "Sampling time must be positive");

    // Initialize to safe default state
    reset();
}

void sfc_mode_detector::configure_mode_handover_speed_pu(speed_pu_t handover_speed) {
    assert(handover_speed.value() >= 0.0 && "Handover speed must be non-negative");
    assert(handover_speed.value() <= 2.0 && "Handover speed should be reasonable (≤ 2.0 pu)");

    _handover_speed_pu = handover_speed;
}

void sfc_mode_detector::configure_hysteresis_factor(real_t hysteresis_factor) {
    assert(hysteresis_factor >= 0.0 && "Hysteresis factor must be non-negative");
    assert(hysteresis_factor <= 0.5 && "Hysteresis factor should be reasonable (≤ 0.5)");

    _hysteresis_factor = hysteresis_factor;
}

void sfc_mode_detector::configure_safety_conditions(duration_t min_enable_time_s,
                                                    current_pu_t max_current_for_switch_pu) {
    assert(min_enable_time_s.value() >= 0.0 && "Minimum enable time must be non-negative");
    assert(max_current_for_switch_pu.value() >= 0.0 && "Maximum current must be non-negative");

    _min_enable_time_s = min_enable_time_s;
    _max_current_for_switch_pu = max_current_for_switch_pu;
}

void sfc_mode_detector::update(bool enable, speed_pu_t measured_speed_pu, current_pu_t measured_dc_current_pu,
                               voltage_pu_t back_emf_magnitude_pu) noexcept {
    // Store current measurements for diagnostics
    _current_speed_pu = measured_speed_pu;
    _current_dc_current_pu = measured_dc_current_pu;
    _current_back_emf_pu = back_emf_magnitude_pu;

    // Update timing counters
    update_timing_counters(enable);

    // Store previous mode for change detection
    _previous_mode = _current_mode;

    // If system is disabled, default to pulse mode
    if (!enable) {
        _current_mode = sfc_operating_mode_t::pulse_mode;
        return;
    }

    // Determine new mode based on current conditions
    sfc_operating_mode_t desired_mode = _current_mode;  // Start with current mode

    // Check if conditions favor Load-Commutated Mode
    if (is_speed_sufficient_for_load_commutated(measured_speed_pu) &&
        is_back_emf_sufficient_for_commutation(back_emf_magnitude_pu) &&
        are_safety_conditions_met(measured_dc_current_pu)) {
        desired_mode = sfc_operating_mode_t::load_commutated;
    } else {
        desired_mode = sfc_operating_mode_t::pulse_mode;
    }

    // Apply mode change (if different from current)
    if (desired_mode != _current_mode) {
        _current_mode = desired_mode;
        _time_since_mode_change = duration_t(0.0);  // Reset mode change timer
    }
}

void sfc_mode_detector::reset() {
    // Reset to safe initial state
    _current_mode = sfc_operating_mode_t::pulse_mode;
    _previous_mode = sfc_operating_mode_t::pulse_mode;
    _time_since_mode_change = duration_t(0.0);
    _time_since_enable = duration_t(0.0);
    _system_enabled = false;

    // Clear measurements
    _current_speed_pu = speed_pu_t(0.0);
    _current_dc_current_pu = current_pu_t(0.0);
    _current_back_emf_pu = voltage_pu_t(0.0);
}

sfc_operating_mode_t sfc_mode_detector::get_operating_mode() const noexcept { return _current_mode; }

sfc_operating_mode_t sfc_mode_detector::get_previous_operating_mode() const noexcept { return _previous_mode; }

bool sfc_mode_detector::has_mode_changed() const noexcept { return _current_mode != _previous_mode; }

duration_t sfc_mode_detector::get_time_since_mode_change() const noexcept { return _time_since_mode_change; }

bool sfc_mode_detector::is_ready_for_load_commutated_mode() const noexcept {
    return is_speed_sufficient_for_load_commutated(_current_speed_pu) &&
           is_back_emf_sufficient_for_commutation(_current_back_emf_pu) &&
           are_safety_conditions_met(_current_dc_current_pu);
}

bool sfc_mode_detector::is_speed_sufficient_for_load_commutated(speed_pu_t measured_speed_pu) const noexcept {
    // Apply hysteresis based on current mode
    if (_current_mode == sfc_operating_mode_t::pulse_mode) {
        // Currently in pulse mode - need higher threshold to switch up
        const real_t switch_up_threshold = _handover_speed_pu.value() * (1.0f + _hysteresis_factor);
        return measured_speed_pu.value() >= switch_up_threshold;
    } else {
        // Currently in load commutated mode - need lower threshold to switch down
        const real_t switch_down_threshold = _handover_speed_pu.value() * (1.0f - _hysteresis_factor);
        return measured_speed_pu.value() >= switch_down_threshold;
    }
}

bool sfc_mode_detector::is_back_emf_sufficient_for_commutation(voltage_pu_t back_emf_magnitude_pu) const noexcept {
    // Back-EMF should be proportional to speed for natural commutation
    // A minimum threshold ensures reliable commutation capability
    constexpr real_t min_back_emf_for_commutation_pu = 0.05f;  // 5% of nominal voltage

    return back_emf_magnitude_pu.value() >= min_back_emf_for_commutation_pu;
}

bool sfc_mode_detector::are_safety_conditions_met(current_pu_t measured_dc_current_pu) const noexcept {
    // Check if system has been enabled long enough
    if (_time_since_enable < _min_enable_time_s) {
        return false;
    }

    // Check if current is within safe limits for mode switching
    if (measured_dc_current_pu.value() > _max_current_for_switch_pu.value()) {
        return false;
    }

    return true;
}

void sfc_mode_detector::update_timing_counters(bool enable) noexcept {
    // Update enable timer
    if (enable) {
        if (!_system_enabled) {
            // System just got enabled - reset enable timer
            _time_since_enable = duration_t(0.0);
        } else {
            // System continues to be enabled - increment timer
            _time_since_enable = duration_t(_time_since_enable.value() + _sampling_time.value());
        }
    } else {
        // System is disabled
        _time_since_enable = duration_t(0.0);
    }

    // Update mode change timer
    _time_since_mode_change = duration_t(_time_since_mode_change.value() + _sampling_time.value());

    // Store enable state for next cycle
    _system_enabled = enable;
}

}  // namespace yeet::application::sfc