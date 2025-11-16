#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_pulse_generator_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_pulse_generator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

sfc_machine_side_pulse_generator_runner::sfc_machine_side_pulse_generator_runner()
    : base_application(sampling_time),
      _pulse_generator(control::six_pulse_generator::reference_type::phase, angle_wrapped::from_degrees(30.0))
{
}

void sfc_machine_side_pulse_generator_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // Configure virtual reference generator with sampling time
    _virtual_reference_generator.configure_sampling_time(sampling_time);
    _virtual_reference_generator.reset();
}

void sfc_machine_side_pulse_generator_runner::run()
{
    // Step 1: Determine the reference angle source
    angle_wrapped effective_reference_angle;
    const bool using_virtual_reference = _parameters.enable_virtual_reference.read();

    if (using_virtual_reference) {
        // Use virtual reference generator
        _virtual_reference_generator.update();
        effective_reference_angle = _virtual_reference_generator.get_angle();
    }
    else {
        // Use normal reference from inputs
        effective_reference_angle = _inputs.reference_angle.read();
    }

    // Step 2: Get firing angle command and enable signals
    const angle_wrapped firing_angle_command = _inputs.firing_angle.read();
    const bool release_pulses_input = _inputs.release_pulses.read();
    const bool enable = _parameters.enable.read();

    // Combine input signal with enable parameter override
    const bool final_release_pulses = release_pulses_input && enable;

    // Step 3: Generate pulses using the six-pulse generator
    control::six_pulse_generator::pulses pulses =
        _pulse_generator.update(effective_reference_angle, firing_angle_command, final_release_pulses);

    // Step 4: Apply individual pulse overrides if enabled
    const bool using_individual_override = _parameters.enable_individual_pulse_override.read();

    if (using_individual_override) {
        pulses.p1 = _parameters.override_p1.read();
        pulses.p2 = _parameters.override_p2.read();
        pulses.p3 = _parameters.override_p3.read();
        pulses.p4 = _parameters.override_p4.read();
        pulses.p5 = _parameters.override_p5.read();
        pulses.p6 = _parameters.override_p6.read();
    }

    // Step 5: Set outputs
    _outputs.firing_pulses = pulses;
    _outputs.actual_firing_angle = firing_angle_command;
    _outputs.release_pulses = final_release_pulses;
    _outputs.virtual_reference_active = using_virtual_reference;
    _outputs.virtual_reference_angle = effective_reference_angle;
    _outputs.individual_override_active = using_individual_override;
}

void sfc_machine_side_pulse_generator_runner::stop()
{
    // Nothing to stop for pulse generator
}

void sfc_machine_side_pulse_generator_runner::set_parameters()
{
    // Configure the pulse generator with parameters
    _pulse_generator.set_pulse_width(_parameters.pulse_width.read());

    // Configure virtual reference generator frequency
    _virtual_reference_generator.configure_frequency(_parameters.virtual_reference_frequency.read());

    // Note: Reference type would need to be set in constructor if the six_pulse_generator
    // supported runtime changes. For now, we assume it's configured at construction.
}

void sfc_machine_side_pulse_generator_runner::set_parameter_callback()
{
    // Register callbacks for parameter changes
    _parameters.pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.reference_type.register_callback([&]() { set_parameters(); });

    // Virtual reference mode callbacks
    _parameters.enable_virtual_reference.register_callback([&]() {
        // Reset virtual reference when mode is toggled
        if (_parameters.enable_virtual_reference.read()) {
            _virtual_reference_generator.reset();
        }
    });
    _parameters.virtual_reference_frequency.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
