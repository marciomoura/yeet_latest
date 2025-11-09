#include "sfc_integration/sfc_startup_sequencer_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_startup_sequencer_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_startup_sequencer_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_startup_sequencer_runner::initialize()
{
    // Reset sequencer to initial state
    _sequencer.reset();

    // Configure all parameters from datamodel
    set_parameters();

    // Register callbacks for dynamic parameter updates
    set_parameter_callback();

    // Initialize command edge detection flags
    _prev_start_command = false;
    _prev_stop_command = false;
}

void sfc_startup_sequencer_runner::run()
{
    // Handle command inputs with edge detection
    handle_commands();

    // Read current field establishment status with override support
    bool is_field_established = _parameters.override_enable_field_established.read()
                                    ? _parameters.override_value_field_established.read()
                                    : _inputs.is_field_established.read();

    // Read estimated rotor angle and convert from angle_wrapped to angle_t
    angle_t estimated_angle = _inputs.estimated_rotor_angle.read().get_radians();

    // Update the sequencer with measured signals
    _sequencer.update(is_field_established, estimated_angle);
    const sfc::startup_sequencer_outputs sequencer_outputs = _sequencer.get_outputs();

    _outputs.current_state.write(sequencer_outputs.controller_commands.current_state);
    _outputs.reference_frame_source.write(sequencer_outputs.controller_commands.reference_frame_source);
    _outputs.torque_control_mode.write(sequencer_outputs.controller_commands.torque_control_mode);
    _outputs.release_machine_side_pulses.write(sequencer_outputs.controller_commands.release_machine_side_pulses);
    _outputs.release_excitation.write(sequencer_outputs.controller_commands.release_excitation);
    _outputs.release_machine_observer.write(sequencer_outputs.controller_commands.release_machine_observer);
    _outputs.release_speed_controller.write(sequencer_outputs.controller_commands.release_speed_controller);
    _outputs.release_torque_controller.write(sequencer_outputs.controller_commands.release_torque_controller);

    _outputs.virtual_rotor_angle.write(sequencer_outputs.virtual_reference.rotor_angle);
    _outputs.virtual_rotor_frequency.write(sequencer_outputs.virtual_reference.rotor_frequency);
    _outputs.virtual_torque_reference.write(sequencer_outputs.virtual_reference.torque_reference);

    _outputs.is_startup_complete.write(_sequencer.is_startup_complete());
    _outputs.time_in_current_state.write(_sequencer.get_time_in_current_state());
}

void sfc_startup_sequencer_runner::stop()
{
    // Return sequencer to safe IDLE state
    _sequencer.stop_sequence();
    _sequencer.reset();
}

void sfc_startup_sequencer_runner::set_parameters()
{
    // Configure virtual frequency phase
    sfc::virtual_frequency_config freq_cfg;
    freq_cfg.initial = _parameters.virtual_frequency_initial.read();
    freq_cfg.final = _parameters.virtual_frequency_final.read();
    freq_cfg.duration = _parameters.virtual_frequency_duration.read();
    _sequencer.configure_virtual_frequency(freq_cfg);

    // Configure virtual torque phase
    sfc::virtual_torque_config torque_cfg;
    torque_cfg.initial = _parameters.virtual_torque_initial.read();
    torque_cfg.final = _parameters.virtual_torque_final.read();
    torque_cfg.duration = _parameters.virtual_torque_duration.read();
    _sequencer.configure_virtual_torque(torque_cfg);
}

void sfc_startup_sequencer_runner::set_parameter_callback()
{
    // Register callbacks for all virtual frequency parameters
    _parameters.virtual_frequency_initial.register_callback([&]() { set_parameters(); });
    _parameters.virtual_frequency_final.register_callback([&]() { set_parameters(); });
    _parameters.virtual_frequency_duration.register_callback([&]() { set_parameters(); });

    // Register callbacks for all virtual torque parameters
    _parameters.virtual_torque_initial.register_callback([&]() { set_parameters(); });
    _parameters.virtual_torque_final.register_callback([&]() { set_parameters(); });
    _parameters.virtual_torque_duration.register_callback([&]() { set_parameters(); });
}

void sfc_startup_sequencer_runner::handle_commands()
{
    // Read current command inputs with override support
    bool current_start_command = _parameters.override_enable_start_command.read()
                                     ? _parameters.override_value_start_command.read()
                                     : _inputs.start_command.read();

    bool current_stop_command = _parameters.override_enable_stop_command.read()
                                    ? _parameters.override_value_stop_command.read()
                                    : _inputs.stop_command.read();

    // Detect rising edge on START command
    if (current_start_command && !_prev_start_command) {
        _sequencer.start_sequence();
    }

    // Detect rising edge on STOP command (has priority)
    if (current_stop_command && !_prev_stop_command) {
        _sequencer.stop_sequence();
    }

    // Update previous command states for next cycle
    _prev_start_command = current_start_command;
    _prev_stop_command = current_stop_command;
}

}  // namespace yeet::application::sfc::integration
