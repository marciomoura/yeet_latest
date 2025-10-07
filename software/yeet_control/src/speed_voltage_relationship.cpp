#include "yeet_control/speed_voltage_relationship.hpp"

#include <algorithm>
#include <cassert>

namespace yeet::control {

speed_voltage_relationship::speed_voltage_relationship(double sampling_time) : _voltage_filter(sampling_time)
{
    assert(sampling_time > 0.0f && "Sampling time must be positive.");
}

void speed_voltage_relationship::configure_voltage_speed_coefficient(real_t coefficient)
{
    assert(coefficient > 0.0f && "Voltage-speed coefficient must be positive.");
    _voltage_speed_coefficient = coefficient;
}

void speed_voltage_relationship::configure_voltage_offset_pu(real_t offset_pu) { _voltage_offset_pu = offset_pu; }

void speed_voltage_relationship::configure_time_constant(real_t time_constant_seconds)
{
    assert(time_constant_seconds >= 0.0f && "Time constant must be non-negative.");
    _time_constant_seconds = time_constant_seconds;
    _is_dynamic_mode = (time_constant_seconds > 0.0f);

    if (_is_dynamic_mode) {
        // Configure the low-pass filter based on the time constant
        // For first-order system: cutoff_frequency = 1/(2π * time_constant)
        const real_t cutoff_frequency =
            first_order_low_pass_filter<real_t>::calculate_cutoff_frequency(time_constant_seconds);
        _voltage_filter.configure(cutoff_frequency);
    }
}

void speed_voltage_relationship::configure_voltage_limits_pu(real_t min_voltage_pu, real_t max_voltage_pu)
{
    assert(min_voltage_pu >= 0.0f && "Minimum voltage must be non-negative.");
    assert(max_voltage_pu > min_voltage_pu && "Maximum voltage must be greater than minimum.");
    _min_voltage_pu = min_voltage_pu;
    _max_voltage_pu = max_voltage_pu;
}

void speed_voltage_relationship::reset(real_t initial_voltage_pu, real_t initial_speed_pu)
{
    const real_t limited_initial_voltage = apply_voltage_limits(initial_voltage_pu);
    _voltage_filter.reset(limited_initial_voltage);
    _current_voltage_pu = limited_initial_voltage;
    _current_speed_pu = initial_speed_pu;  // Reset speed to zero
}

void speed_voltage_relationship::update(real_t speed_pu) noexcept
{
    _current_speed_pu = speed_pu;

    const real_t steady_state_voltage = calculate_steady_state_voltage(speed_pu);

    if (_is_dynamic_mode) {
        // Dynamic mode: use low-pass filter to create first-order response
        _current_voltage_pu = _voltage_filter.update(steady_state_voltage);

        // Apply limits to the filtered result
        _current_voltage_pu = apply_voltage_limits(_current_voltage_pu);
    }
    else {
        // Algebraic mode: direct calculation
        _current_voltage_pu = apply_voltage_limits(steady_state_voltage);
    }
}

real_t speed_voltage_relationship::get_voltage_magnitude_pu() const noexcept { return _current_voltage_pu; }

real_t speed_voltage_relationship::get_steady_state_voltage_pu() const noexcept
{
    return apply_voltage_limits(calculate_steady_state_voltage(_current_speed_pu));
}

void speed_voltage_relationship::reset(real_t initial_voltage_pu)
{
    const real_t limited_initial_voltage = apply_voltage_limits(initial_voltage_pu);
    _voltage_filter.reset(limited_initial_voltage);
    _current_voltage_pu = limited_initial_voltage;
    _current_speed_pu = 0.0f;
}

bool speed_voltage_relationship::is_dynamic_mode() const noexcept { return _is_dynamic_mode; }

real_t speed_voltage_relationship::calculate_steady_state_voltage(real_t speed_pu) const noexcept
{
    return _voltage_speed_coefficient * speed_pu + _voltage_offset_pu;
}

real_t speed_voltage_relationship::apply_voltage_limits(real_t voltage_pu) const noexcept
{
    return std::clamp(voltage_pu, _min_voltage_pu, _max_voltage_pu);
}

}  // namespace yeet::control