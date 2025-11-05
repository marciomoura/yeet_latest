#include "sfc/sfc_excitation_controller.hpp"

#include <algorithm>
#include <cassert>

namespace yeet::application::sfc {

sfc_excitation_controller::sfc_excitation_controller(duration_t sampling_time)
    : _sampling_time(sampling_time),
      _excitation_voltage_rate_limiter(10000.0, sampling_time.value()),  // High default rate for fast tracking
      _prefluxing_timer(sampling_time)
{
    // Configure default lookup table with safe initial values
    _excitation_voltage_lookup_table.configure(kDefaultSpeedAxisPu, kDefaultExcitationVoltageSetpointPu);
}

void sfc_excitation_controller::configure_excitation_voltage_lookup_table(
    const std::array<real_t, NUM_LOOKUP_POINTS>& speed_axis_pu,
    const std::array<real_t, NUM_LOOKUP_POINTS>& excitation_voltage_setpoint)
{
    // Verify that speed axis is sorted
    assert(
        std::is_sorted(speed_axis_pu.begin(), speed_axis_pu.end()) && "Speed axis must be sorted in ascending order.");

    // Verify that all excitation voltages are non-negative
    for (const auto& voltage : excitation_voltage_setpoint) {
        assert(voltage >= 0.0 && "Excitation voltage setpoint values must be non-negative.");
    }

    _excitation_voltage_lookup_table.configure(speed_axis_pu, excitation_voltage_setpoint);
}

void sfc_excitation_controller::configure_prefluxing_duration(duration_t prefluxing_duration)
{
    assert(prefluxing_duration.value() > 0.0 && "Pre-fluxing duration must be positive.");
    if (prefluxing_duration.value() <= 0.0) {
        return;
    }

    _prefluxing_duration = prefluxing_duration;
}

void sfc_excitation_controller::configure_field_established_timeout(duration_t timeout)
{
    assert(timeout.value() > 0.0 && "Field established timeout must be positive.");
    if (timeout.value() <= 0.0) {
        return;
    }

    _field_established_timeout = timeout;
}

void sfc_excitation_controller::configure_excitation_ramp_down_time(duration_t ramp_down_time)
{
    assert(ramp_down_time.value() > 0.0 && "Ramp down time must be positive.");
    if (ramp_down_time.value() <= 0.0) {
        return;
    }

    // Configure rate limiter for excitation voltage ramp-down
    // Calculate rate in pu/s needed to reach zero from max voltage in specified time
    // Using a conservative estimate of max excitation voltage from lookup table
    const real_t max_excitation_voltage =
        *std::max_element(kDefaultExcitationVoltageSetpointPu.begin(), kDefaultExcitationVoltageSetpointPu.end());
    const real_t ramp_rate = max_excitation_voltage / ramp_down_time.value();

    _excitation_voltage_rate_limiter.configure(ramp_rate);
}

bool sfc_excitation_controller::start_excitation()
{
    // Precondition: controller must not already be running
    if (_is_running) {
        return false;
    }

    // Start the excitation sequence
    _is_running = true;
    _is_field_established = false;
    _is_ramping_down = false;
    _enable_excitation = true;

    // Set excitation voltage setpoint to initial value (0% speed point from lookup table)
    _excitation_voltage_setpoint_pu = voltage_pu_t{kDefaultExcitationVoltageSetpointPu[0]};

    // Start pre-fluxing timer
    _prefluxing_timer.reset();

    return true;
}

void sfc_excitation_controller::stop_excitation()
{
    // Initiate controlled ramp-down
    _is_ramping_down = true;
    _is_field_established = false;

    // Note: actual ramp-down and disable happens in update()
}

void sfc_excitation_controller::update(speed_pu_t machine_rotor_speed_pu, bool exciter_ready)
{
    // Store last valid speed for fault handling
    if (machine_rotor_speed_pu.value() >= 0.0) {
        _last_valid_speed = machine_rotor_speed_pu;
    }

    // Handle exciter fault - stop immediately if exciter not ready while running
    if (_is_running && !exciter_ready) {
        stop_excitation();
    }

    // If not running, ensure outputs are disabled
    if (!_is_running && !_is_ramping_down) {
        _excitation_voltage_setpoint_pu = voltage_pu_t{0.0};
        _enable_excitation = false;
        return;
    }

    // Handle ramp-down sequence
    if (_is_ramping_down) {
        // Ramp excitation voltage to zero using rate limiter
        _excitation_voltage_rate_limiter.update(0.0);
        _excitation_voltage_setpoint_pu = voltage_pu_t{_excitation_voltage_rate_limiter.get_output()};

        // Once reached zero, disable excitation and clear running flag
        if (_excitation_voltage_setpoint_pu.value() <= 0.01) {  // Small threshold for numerical stability
            _excitation_voltage_setpoint_pu = voltage_pu_t{0.0};
            _enable_excitation = false;
            _is_running = false;
            _is_ramping_down = false;
            _prefluxing_timer.reset();
        }
        return;
    }

    // Pre-fluxing phase: maintain initial excitation voltage and wait for timer
    if (!_is_field_established) {
        // Update pre-fluxing timer
        _prefluxing_timer.update(true);

        // Check if pre-fluxing duration has elapsed
        if (_prefluxing_timer.has_elapsed(_prefluxing_duration)) {
            _is_field_established = true;
        }

        // Maintain initial excitation voltage (0% speed point)
        _excitation_voltage_setpoint_pu = voltage_pu_t{kDefaultExcitationVoltageSetpointPu[0]};
        _enable_excitation = true;
        return;
    }

    // Normal operation: field established, update from lookup table
    // Evaluate lookup table with current rotor speed
    const real_t excitation_voltage_setpoint =
        _excitation_voltage_lookup_table.get_value(machine_rotor_speed_pu.value());

    // Apply rate limiting for smooth transitions
    _excitation_voltage_rate_limiter.update(excitation_voltage_setpoint);
    _excitation_voltage_setpoint_pu = voltage_pu_t{_excitation_voltage_rate_limiter.get_output()};

    // Maintain excitation enable
    _enable_excitation = true;
}

void sfc_excitation_controller::reset()
{
    _is_running = false;
    _is_field_established = false;
    _is_ramping_down = false;
    _enable_excitation = false;
    _excitation_voltage_setpoint_pu = voltage_pu_t{0.0};
    _last_valid_speed = speed_pu_t{0.0};

    _excitation_voltage_rate_limiter.reset(0.0);
    _prefluxing_timer.reset();
}

voltage_pu_t sfc_excitation_controller::get_excitation_voltage_setpoint_pu() const noexcept
{
    return _excitation_voltage_setpoint_pu;
}

bool sfc_excitation_controller::get_enable_excitation() const noexcept { return _enable_excitation; }

bool sfc_excitation_controller::get_is_field_established() const noexcept { return _is_field_established; }

bool sfc_excitation_controller::get_is_running() const noexcept { return _is_running; }

duration_t sfc_excitation_controller::get_prefluxing_elapsed_time() const noexcept
{
    return _prefluxing_timer.get_elapsed_time();
}

}  // namespace yeet::application::sfc
