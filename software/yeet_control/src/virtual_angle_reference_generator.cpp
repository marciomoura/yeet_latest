#include "yeet_control/virtual_angle_reference_generator.hpp"

#include "yeet_control/constants.hpp"

namespace yeet::control {

void virtual_angle_reference_generator::configure_frequency(frequency_t frequency) { _frequency = frequency; }

void virtual_angle_reference_generator::configure_sampling_time(duration_t sampling_time)
{
    _sampling_time = sampling_time;
}

void virtual_angle_reference_generator::update() noexcept
{
    // Integrate angle: angle += 2π × frequency × dt
    // angle_wrapped automatically handles wrapping at 2π
    const angle_t angular_increment = two_pi * _frequency * _sampling_time;
    _angle += angular_increment;
}

void virtual_angle_reference_generator::reset() noexcept { _angle = angle_wrapped{0.0}; }

void virtual_angle_reference_generator::reset(angle_wrapped initial_angle) noexcept { _angle = initial_angle; }

angle_wrapped virtual_angle_reference_generator::get_angle() const noexcept { return _angle; }

frequency_t virtual_angle_reference_generator::get_frequency() const noexcept { return _frequency; }

duration_t virtual_angle_reference_generator::get_sampling_time() const noexcept { return _sampling_time; }

}  // namespace yeet::control
