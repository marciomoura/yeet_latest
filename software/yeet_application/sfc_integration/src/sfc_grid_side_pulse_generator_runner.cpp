#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_pulse_generator_runner::sfc_grid_side_pulse_generator_runner() : base_application(sampling_time) {}

void sfc_grid_side_pulse_generator_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
}

void sfc_grid_side_pulse_generator_runner::run()
{
    // Get inputs
    const angle_wrapped firing_angle_command = _inputs.firing_angle.read();
    const angle_wrapped reference_angle = _inputs.reference_angle.read();
    const bool enable = _parameters.enable.read() && _inputs.release_pulses.read();

    // Generate pulses using the six-pulse generator
    control::six_pulse_generator::pulses pulses =
        _pulse_generator.update(reference_angle, firing_angle_command, enable);

    // Set outputs
    _outputs.firing_pulses = pulses;
    _outputs.actual_firing_angle = firing_angle_command;
}

void sfc_grid_side_pulse_generator_runner::stop()
{
    // Nothing to stop for pulse generator
}

void sfc_grid_side_pulse_generator_runner::set_parameters()
{
    // Configure the pulse generator with parameters
    _pulse_generator.set_pulse_width(_parameters.pulse_width.read());

    // Note: Reference type would need to be set in constructor if the six_pulse_generator
    // supported runtime changes. For now, we assume it's configured at construction.
}

void sfc_grid_side_pulse_generator_runner::set_parameter_callback()
{
    // Register callbacks for parameter changes
    _parameters.min_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.max_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.reference_type.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
