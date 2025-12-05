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

    // Configure virtual reference generator with sampling time
    _pulse_generator.configure_virtual_reference(_parameters.virtual_reference_frequency.read(), sampling_time);
    _pulse_generator.reset_virtual_reference();
}

void sfc_grid_side_pulse_generator_runner::run()
{
    // Read mode flags from parameters
    const bool using_individual_override = _parameters.enable_individual_pulse_override.read();
    const bool using_virtual_reference = _parameters.enable_virtual_reference.read();

    // Get firing angle command
    const angle_wrapped firing_angle_command = _inputs.firing_angle.read();
    const bool release_pulses_input =
        _inputs.release_pulses.read() || using_virtual_reference || using_individual_override;

    // Configure pulse masking if individual override mode is enabled
    if (using_individual_override) {
        control::six_pulse_generator::pulse_mask mask{_parameters.override_p1.read(), _parameters.override_p2.read(),
            _parameters.override_p3.read(), _parameters.override_p4.read(), _parameters.override_p5.read(),
            _parameters.override_p6.read()};
        _pulse_generator.configure_pulse_mask(mask);
        _pulse_generator.enable_pulse_mask(true);
    }
    else {
        _pulse_generator.enable_pulse_mask(false);
    }

    // Generate pulses based on operating mode
    control::six_pulse_generator::pulses pulses;
    angle_wrapped effective_reference_angle;

    if (using_individual_override || using_virtual_reference) {
        // Use virtual reference mode (internal angle integration)
        _pulse_generator.enable_virtual_reference(true);
        pulses = _pulse_generator.update(firing_angle_command, release_pulses_input);
        effective_reference_angle = _pulse_generator.get_virtual_reference_angle();
    }
    else {
        // Use normal mode with external reference angle from PLL
        _pulse_generator.enable_virtual_reference(false);
        effective_reference_angle = _inputs.reference_angle.read();
        pulses = _pulse_generator.update(effective_reference_angle, firing_angle_command, release_pulses_input);
    }

    // Set outputs
    _outputs.firing_pulses = pulses;
    _outputs.actual_firing_angle = firing_angle_command;
    _outputs.release_pulses = release_pulses_input;
    _outputs.virtual_reference_active = using_virtual_reference || using_individual_override;
    _outputs.virtual_reference_angle = effective_reference_angle;
    _outputs.individual_override_active = using_individual_override;
}

void sfc_grid_side_pulse_generator_runner::stop()
{
    // Nothing to stop for pulse generator
}

void sfc_grid_side_pulse_generator_runner::set_parameters()
{
    // Configure the pulse generator with parameters
    _pulse_generator.set_pulse_width(_parameters.pulse_width.read());

    // Configure virtual reference generator frequency
    _pulse_generator.configure_virtual_reference(_parameters.virtual_reference_frequency.read(), sampling_time);
}

void sfc_grid_side_pulse_generator_runner::set_parameter_callback()
{
    // Register callbacks for parameter changes
    _parameters.pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.reference_type.register_callback([&]() {
        // Note: Reference type requires re-initialization. For now, we assume it's set at construction.
    });

    // Virtual reference mode callbacks
    _parameters.enable_virtual_reference.register_callback([&]() {
        // Reset virtual reference when mode is toggled
        if (_parameters.enable_virtual_reference.read()) {
            _pulse_generator.reset_virtual_reference();
        }
    });
    _parameters.virtual_reference_frequency.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
