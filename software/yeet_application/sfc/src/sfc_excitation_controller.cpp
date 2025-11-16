#include "sfc/sfc_excitation_controller.hpp"

#include <algorithm>
#include <cassert>

namespace yeet::application::sfc {

sfc_excitation_controller::sfc_excitation_controller(duration_t sampling_time)
    : _sampling_time(sampling_time),
      // _field_current_rate_limiter(10000.0, sampling_time.value()),  // Commented out - not used in simplified version
      _flux_buildup_timer(sampling_time)
{
    // Simplified constructor - no lookup table initialization needed
    // _field_current_lookup_table.configure(kDefaultSpeedAxisPu, kDefaultFieldCurrentReferencePu);
}

void sfc_excitation_controller::configure_field_current_setpoint(current_pu_t field_current_setpoint)
{
    assert(field_current_setpoint.value() >= 0.0 && "Field current setpoint must be non-negative.");

    _field_current_setpoint = field_current_setpoint;

    // Commented out - simplified version doesn't use lookup table
    // std::array<real_t, NUM_LOOKUP_POINTS> constant_table;
    // constant_table.fill(field_current_setpoint.value());
    // _field_current_lookup_table.configure(kDefaultSpeedAxisPu, constant_table);
}

// Commented out - not used in simplified version
// void sfc_excitation_controller::configure_constant_flux_speed_threshold(speed_pu_t speed_threshold_pu)
// {
//     assert(speed_threshold_pu.value() >= 0.0 && speed_threshold_pu.value() <= 1.0 &&
//            "Constant flux speed threshold must be between 0.0 and 1.0 pu.");
//     if (speed_threshold_pu.value() < 0.0 || speed_threshold_pu.value() > 1.0) {
//         return;
//     }
//
//     _constant_flux_speed_threshold = speed_threshold_pu;
// }

void sfc_excitation_controller::configure_flux_buildup_duration(duration_t flux_buildup_duration)
{
    assert(flux_buildup_duration.value() > 0.0 && "Flux build-up duration must be positive.");
    if (flux_buildup_duration.value() <= 0.0) {
        return;
    }

    _flux_buildup_duration = flux_buildup_duration;
}

bool sfc_excitation_controller::start_excitation()
{
    // Simplified: just enable and set constant field current
    _is_running = true;
    _is_field_established = false;
    _enable_excitation = true;

    // Set field current reference to configured constant value
    _field_current_reference_pu = _field_current_setpoint;

    // Start flux build-up timer
    _flux_buildup_timer.reset();

    // Commented out - simplified version doesn't track ramp-down state
    // _is_ramping_down = false;

    return true;
}

void sfc_excitation_controller::stop_excitation()
{
    // Simplified: immediate disable, no ramp-down
    // _is_running = false;
    // _is_field_established = false;
    // _enable_excitation = false;
    // _field_current_reference_pu = current_pu_t{0.0};
    // _flux_buildup_timer.reset();

    // Commented out - simplified version doesn't use ramp-down
    // _is_ramping_down = true;
}

void sfc_excitation_controller::update(bool enable_excitation_command)
{
    // Handle enable/disable command directly
    if (!enable_excitation_command) {
        // Excitation disabled - reset all states
        _is_running = false;
        _is_field_established = false;
        _enable_excitation = false;
        _field_current_reference_pu = current_pu_t{0.0};
        _flux_buildup_timer.reset();
        return;
    }

    // Excitation enabled - detect rising edge to reset timer
    if (!_is_running) {
        _is_running = true;
        _is_field_established = false;
        _flux_buildup_timer.reset();
    }

    // Commented out - simplified version doesn't use ramp-down
    // if (_is_ramping_down) {
    //     _field_current_rate_limiter.update(0.0);
    //     _field_current_reference_pu = current_pu_t{_field_current_rate_limiter.get_output()};
    //     if (_field_current_reference_pu.value() <= 0.01) {
    //         _field_current_reference_pu = current_pu_t{0.0};
    //         _enable_excitation = false;
    //         _is_running = false;
    //         _is_ramping_down = false;
    //         _prefluxing_timer.reset();
    //     }
    //     return;
    // }

    // Flux build-up phase: maintain constant field current and wait for timer
    if (!_is_field_established) {
        // Update flux build-up timer
        _flux_buildup_timer.update(true);

        // Check if flux build-up duration has elapsed
        if (_flux_buildup_timer.has_elapsed(_flux_buildup_duration)) {
            _is_field_established = true;
        }
    }

    // Normal operation: field established, maintain constant field current
    _field_current_reference_pu = _field_current_setpoint;
    _enable_excitation = true;

    // Commented out - simplified version doesn't use lookup table or rate limiter
    // real_t field_current_reference;
    // if (machine_rotor_speed_pu.value() < _constant_flux_speed_threshold.value()) {
    //     field_current_reference = kDefaultFieldCurrentReferencePu[0];
    // }
    // else {
    //     field_current_reference = _field_current_lookup_table.get_value(machine_rotor_speed_pu.value());
    // }
    // _field_current_rate_limiter.update(field_current_reference);
    // _field_current_reference_pu = current_pu_t{_field_current_rate_limiter.get_output()};
}

void sfc_excitation_controller::reset()
{
    _is_running = false;
    _is_field_established = false;
    _enable_excitation = false;
    _field_current_reference_pu = current_pu_t{0.0};

    _flux_buildup_timer.reset();

    // Commented out - simplified version doesn't use these
    // _is_ramping_down = false;
    // _last_valid_speed = speed_pu_t{0.0};
    // _field_current_rate_limiter.reset(0.0);
}

current_pu_t sfc_excitation_controller::get_field_current_reference_pu() const noexcept
{
    return _field_current_reference_pu;
}

bool sfc_excitation_controller::get_enable_excitation() const noexcept { return _enable_excitation; }

bool sfc_excitation_controller::get_is_field_established() const noexcept { return _is_field_established; }

bool sfc_excitation_controller::get_is_running() const noexcept { return _is_running; }

duration_t sfc_excitation_controller::get_flux_buildup_elapsed_time() const noexcept
{
    return _flux_buildup_timer.get_elapsed_time();
}

// Commented out - legacy compatibility methods not used in simplified version
// void sfc_excitation_controller::configure_field_current_lookup_table(
//     const std::array<real_t, NUM_LOOKUP_POINTS>& speed_axis_pu,
//     const std::array<real_t, NUM_LOOKUP_POINTS>& field_current_reference)
// {
//     assert(
//         std::is_sorted(speed_axis_pu.begin(), speed_axis_pu.end()) && "Speed axis must be sorted in ascending
//         order.");
//
//     for (const auto& current : field_current_reference) {
//         assert(current >= 0.0 && "Field current reference values must be non-negative.");
//     }
//
//     _field_current_lookup_table.configure(speed_axis_pu, field_current_reference);
// }
//
// void sfc_excitation_controller::configure_field_established_timeout(duration_t timeout)
// {
//     assert(timeout.value() > 0.0 && "Field established timeout must be positive.");
//     if (timeout.value() <= 0.0) {
//         return;
//     }
//
//     _field_established_timeout = timeout;
// }
//
// void sfc_excitation_controller::configure_field_current_ramp_down_time(duration_t ramp_down_time)
// {
//     assert(ramp_down_time.value() > 0.0 && "Ramp down time must be positive.");
//     if (ramp_down_time.value() <= 0.0) {
//         return;
//     }
//
//     const real_t max_field_current =
//         *std::max_element(kDefaultFieldCurrentReferencePu.begin(), kDefaultFieldCurrentReferencePu.end());
//     const real_t ramp_rate = max_field_current / ramp_down_time.value();
//
//     _field_current_rate_limiter.configure(ramp_rate);
// }

}  // namespace yeet::application::sfc
