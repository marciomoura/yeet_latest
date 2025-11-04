#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_pulse_generator_runner::sfc_grid_side_pulse_generator_runner()
    : base_application(sampling_time),
      _pulse_generator(control::six_pulse_generator::reference_type::phase, angle_wrapped::from_degrees(30.0))
{
}

void sfc_grid_side_pulse_generator_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_pulse_generator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

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
    const bool release_pulses_input = _inputs.release_pulses.read();
    const bool enable = _parameters.enable.read();

    // Combine input signal with enable parameter override
    const bool final_release_pulses = release_pulses_input && enable;

    // Generate pulses using the six-pulse generator
    control::six_pulse_generator::pulses pulses =
        _pulse_generator.update(reference_angle, firing_angle_command, final_release_pulses);

    // Set outputs
    _outputs.firing_pulses = pulses;
    _outputs.actual_firing_angle = firing_angle_command;
    _outputs.release_pulses = final_release_pulses;
}

void sfc_grid_side_pulse_generator_runner::stop()
{
    // Nothing to stop for pulse generator
}

void sfc_grid_side_pulse_generator_runner::set_parameters()
{
    // Configure the pulse generator with parameters
    _pulse_generator.set_pulse_width(_parameters.pulse_width.read());

    // Note: Reference type cannot be changed at runtime in the current six_pulse_generator implementation.
    // It is set to a default value in the constructor and would need re-initialization to change.
}

void sfc_grid_side_pulse_generator_runner::set_parameter_callback()
{
    // Register callbacks for parameter changes
    _parameters.pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.reference_type.register_callback([&]() {
        // Note: This requires re-initialization if we want to support it.
        // For now, we assume it's set at construction.
    });
}

}  // namespace yeet::application::sfc::integration
