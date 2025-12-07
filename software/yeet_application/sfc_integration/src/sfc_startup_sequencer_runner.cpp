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

    // Ensure speed_reference is at least virtual_frequency_final on startup
    clamp_speed_reference_to_virtual_frequency_final();

    // Register callbacks for dynamic parameter updates
    set_parameter_callback();
}

void sfc_startup_sequencer_runner::run()
{
    // Read current field establishment status with override support
    bool is_field_established = _parameters.field_established_override.read()
                                    ? _parameters.field_established_override_value.read()
                                    : _inputs.is_field_established.read();

    // Read estimated rotor angle and convert from angle_wrapped to angle_t
    angle_t estimated_angle = _inputs.estimated_rotor_angle.read().get_radians();

    // Read measured speed from observer
    speed_pu_t measured_speed = _inputs.measured_speed.read();

    // Read measured voltage magnitude from observer for flying restart detection
    voltage_pu_t measured_voltage_magnitude = _inputs.machine_voltage_magnitude.read();

    // Read current state ID from grid-side state machine
    sfc_state_id current_state_id = _inputs.current_state_id.read();

    // Read speed reference from parameter
    speed_pu_t speed_reference = _parameters.speed_reference.read();

    // Get the base frequency from the per-unit system
    frequency_t base_frequency = _per_unit_system->get_machine_bases().base_frequency.value();

    // Update the sequencer with measured signals
    _sequencer.update(is_field_established, estimated_angle, measured_speed, measured_voltage_magnitude,
        current_state_id, speed_reference, base_frequency);
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
    _outputs.speed_reference.write(sequencer_outputs.speed_control.speed_reference);

    _outputs.is_startup_complete.write(_sequencer.is_startup_complete());
    _outputs.time_in_current_state.write(_sequencer.get_time_in_current_state());

    // Write preset signals
    _outputs.preset_observer.write(sequencer_outputs.controller_commands.preset_observer);
    _outputs.preset_angle.write(sequencer_outputs.controller_commands.preset_angle);
    _outputs.preset_frequency.write(sequencer_outputs.controller_commands.preset_frequency);
}

void sfc_startup_sequencer_runner::stop()
{
    // Return sequencer to safe IDLE state
    _sequencer.stop_sequence();
    _sequencer.reset();
}

void sfc_startup_sequencer_runner::set_parameters()
{
    // Configure virtual frequency phase (already in per-unit from datamodel)
    sfc::virtual_frequency_config freq_cfg;
    freq_cfg.initial = _parameters.virtual_frequency_initial.read();
    freq_cfg.final = _parameters.virtual_frequency_final.read();
    freq_cfg.duration = _parameters.virtual_frequency_duration.read();
    _sequencer.configure_virtual_frequency(freq_cfg);

    // Configure virtual torque phase (already in per-unit)
    sfc::virtual_torque_config torque_cfg;
    torque_cfg.initial = _parameters.virtual_torque_initial.read();
    torque_cfg.final = _parameters.virtual_torque_final.read();
    torque_cfg.duration = _parameters.virtual_torque_duration.read();
    _sequencer.configure_virtual_torque(torque_cfg);

    // Configure speed reference rate limit
    sfc::speed_reference_rate_limit_config rate_limit_cfg;
    rate_limit_cfg.seconds_per_pu = _parameters.speed_reference_rate_limit_seconds_per_pu.read();
    rate_limit_cfg.enabled = _parameters.speed_reference_rate_limit_enabled.read();
    _sequencer.configure_speed_reference_rate_limit(rate_limit_cfg);

    // Configure flying restart
    sfc::flying_restart_config flying_restart_cfg;
    flying_restart_cfg.speed_threshold = _parameters.flying_restart_speed_threshold.read();
    flying_restart_cfg.voltage_threshold = _parameters.flying_restart_voltage_threshold.read();
    flying_restart_cfg.observer_settling_time = _parameters.flying_restart_observer_settling_time.read();
    flying_restart_cfg.auto_start_stop_enabled = _parameters.auto_start_stop_enabled.read();
    _sequencer.configure_flying_restart(flying_restart_cfg);
}

void sfc_startup_sequencer_runner::set_parameter_callback()
{
    // Register callbacks for all virtual frequency parameters
    _parameters.virtual_frequency_initial.register_callback([&]() { set_parameters(); });
    _parameters.virtual_frequency_final.register_callback([&]() {
        set_parameters();
        clamp_speed_reference_to_virtual_frequency_final();
    });
    _parameters.virtual_frequency_duration.register_callback([&]() { set_parameters(); });

    // Register callbacks for all virtual torque parameters
    _parameters.virtual_torque_initial.register_callback([&]() { set_parameters(); });
    _parameters.virtual_torque_final.register_callback([&]() { set_parameters(); });
    _parameters.virtual_torque_duration.register_callback([&]() { set_parameters(); });

    // Register callbacks for speed reference rate limit parameters
    _parameters.speed_reference_rate_limit_seconds_per_pu.register_callback([&]() { set_parameters(); });
    _parameters.speed_reference_rate_limit_enabled.register_callback([&]() { set_parameters(); });

    // Register callbacks for flying restart parameters
    _parameters.flying_restart_speed_threshold.register_callback([&]() { set_parameters(); });
    _parameters.flying_restart_voltage_threshold.register_callback([&]() { set_parameters(); });
    _parameters.flying_restart_observer_settling_time.register_callback([&]() { set_parameters(); });
    _parameters.auto_start_stop_enabled.register_callback([&]() { set_parameters(); });
}

void sfc_startup_sequencer_runner::clamp_speed_reference_to_virtual_frequency_final()
{
    const frequency_pu_t virtual_freq_final = _parameters.virtual_frequency_final.read();
    const speed_pu_t current_speed_ref = _parameters.speed_reference.read();

    // Only clamp if speed_reference is below virtual_frequency_final
    if (current_speed_ref.value() < virtual_freq_final.value()) {
        _parameters.speed_reference.write(speed_pu_t{virtual_freq_final.value()});
    }
}

}  // namespace yeet::application::sfc::integration
