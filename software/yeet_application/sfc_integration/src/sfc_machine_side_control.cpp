#include "sfc_integration/sfc_machine_side_control.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_control::initialize()
{
    excitation_controller.initialize();
    speed_generator.initialize();
    speed_controller.initialize();
    torque_controller.initialize();
    startup_sequencer.initialize();
    machine_integrator_supervisor.initialize();
    machine_side_controller.initialize();
    machine_side_observer.initialize();
    machine_side_modulator.initialize();
    machine_side_pulse_generator.initialize();
    machine_measurements.initialize();
}

void sfc_machine_side_control::update_10ms()
{
    // 10ms sampling: Execute in order: supervisor → generator → controller → limiter
    machine_integrator_supervisor.run();  // 0. Coordinate freeze/preset/trip logic
    // speed_generator.run();                // 1. Generate speed reference + dynamic torque limits
    speed_controller.run();   // 2. Process speed error → torque reference
    torque_controller.run();  // 3. Apply limits + boost → current reference
}

void sfc_machine_side_control::update_1ms()
{
    // 1ms sampling: Excitation controller, startup sequencer, machine-side controller
    excitation_controller.run();
    startup_sequencer.run();
    machine_side_controller.run();
}

void sfc_machine_side_control::update_200us()
{
    // 200µs sampling: Measurements, observers, modulators
    machine_measurements.run();
    machine_side_observer.run();
    machine_side_modulator.run();
    machine_side_pulse_generator.run();
}

std::uint8_t sfc_machine_side_control::encode_fiber_optic_tx_to_grid()
{
    fiber_optic::machine_to_grid_message tx_msg;
    tx_msg.measured_speed = machine_side_observer.get_outputs().estimated_speed.read().value();
    tx_msg.dc_link_current_reference_pu = torque_controller.get_outputs().current_reference.read().value();
    tx_msg.machine_voltage_magnitude_pu = machine_measurements.get_outputs().machine_voltage_magnitude.read().value();
    tx_msg.force_commutation_request = machine_side_modulator.get_outputs().request_force_commutation.read();
    tx_msg.shutdown_request = machine_integrator_supervisor.get_outputs().request_shutdown.read();
    _fiber_optic_tx_length = fiber_optic::encode_machine_to_grid(tx_msg, _fiber_optic_tx_payload.data());
    return _fiber_optic_tx_length;
}

bool sfc_machine_side_control::decode_fiber_optic_rx_from_grid(
    const std::uint32_t* payload, std::uint8_t payload_length)
{
    auto rx_msg = fiber_optic::decode_grid_to_machine(payload, payload_length);
    if (!rx_msg) return false;

    // All signals are now extracted directly from the fiber-optic protocol
    external_ios.update_fiber_optics_inputs(rx_msg->mode, static_cast<sfc_state_id>(rx_msg->state_id),
        rx_msg->force_commutation_acknowledged, rx_msg->current_is_zero, rx_msg->is_saturated, rx_msg->pulse_releases,
        current_pu_t{rx_msg->dc_link_current}, rx_msg->preset_request);
    return true;
}

void sfc_machine_side_control::register_datamodel(yeet::model_registry& registry)
{
    // Register all component parameters with the model_registry
    // This makes them accessible via the parameter configuration protocol

    // Register per-unit system parameters first (fundamental base quantities)
    pu_system_datamodel.register_datamodel(registry, per_unit_system);
    // Register external I/O overrides datamodel
    external_ios.register_datamodel(registry);

    machine_integrator_supervisor.register_datamodel(registry);
    excitation_controller.register_datamodel(registry);
    speed_generator.register_datamodel(registry);
    speed_controller.register_datamodel(registry);
    torque_controller.register_datamodel(registry);
    startup_sequencer.register_datamodel(registry);
    machine_side_controller.register_datamodel(registry);
    machine_side_observer.register_datamodel(registry);
    machine_side_modulator.register_datamodel(registry);
    machine_side_pulse_generator.register_datamodel(registry);
    machine_measurements.register_datamodel(registry);
}

void sfc_machine_side_control::connect_internal_signals()
{
    // ==================== Signal Connections Organized by Receiving Component ====================

    // ==================== Machine Measurements Inputs ====================
    signals_connector::connect_output_to_input(
        external_ios.get_analog_inputs().v_ab_counts, machine_measurements.get_inputs().v_ab_counts);
    signals_connector::connect_output_to_input(
        external_ios.get_analog_inputs().v_bc_counts, machine_measurements.get_inputs().v_bc_counts);
    signals_connector::connect_output_to_input(
        machine_side_pulse_generator.get_outputs().firing_pulses, machine_measurements.get_inputs().machine_pulses);
    signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_reference_angle,
        machine_measurements.get_inputs().machine_reference_angle);
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_frequency, machine_measurements.get_inputs().machine_frequency);

    // ==================== Machine-Side Observer Inputs ====================
    signals_connector::connect_output_to_input(
        machine_measurements.get_outputs().machine_voltage_abc, machine_side_observer.get_inputs().machine_voltage_pu);
    // Observer enable from startup sequencer
    signals_connector::connect_output_to_input(
        startup_sequencer.get_outputs().release_machine_observer, machine_side_observer.get_inputs().release_observer);

    // ==================== Machine Integrator Supervisor Inputs ====================
    // Connect fiber-optic signals from grid-side
    signals_connector::connect_output_to_input(external_ios.get_fiber_optics_inputs().pulse_releases,
        machine_integrator_supervisor.get_inputs().grid_side_release_pulses);

    // Connect saturation signal for anti-windup (only torque controller)
    signals_connector::connect_output_to_input(
        torque_controller.get_outputs().is_saturated, machine_integrator_supervisor.get_inputs().torque_saturated);

    // ==================== Excitation Controller Inputs ====================
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, excitation_controller.get_inputs().machine_rotor_speed_pu);
    // Excitation enable from startup sequencer
    signals_connector::connect_output_to_input(
        startup_sequencer.get_outputs().release_excitation, excitation_controller.get_inputs().release_excitation);

    // ==================== Startup Sequencer Inputs ====================
    signals_connector::connect_output_to_input(
        excitation_controller.get_outputs().is_field_established, startup_sequencer.get_inputs().is_field_established);
    signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_reference_angle,
        startup_sequencer.get_inputs().estimated_rotor_angle);
    signals_connector::connect_output_to_input(
        external_ios.get_startup_sequencer_outputs().start_command, startup_sequencer.get_inputs().start_command);
    signals_connector::connect_output_to_input(
        external_ios.get_startup_sequencer_outputs().stop_command, startup_sequencer.get_inputs().stop_command);
    // External speed reference input for rate limiting
    signals_connector::connect_output_to_input(
        external_ios.get_fixed_speed_reference(), startup_sequencer.get_inputs().external_speed_reference);

    // ==================== Speed Generator Inputs (TEMPORARILY DISABLED) ====================
    // Speed generator disconnected during startup sequencer testing
    // Will be re-integrated later for multi-profile speed control
    // signals_connector::connect_output_to_input(
    //     machine_side_observer.get_outputs().estimated_speed, speed_generator.get_inputs().measured_speed);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_fiber_optics_inputs().current_state_id, speed_generator.get_inputs().current_state_id);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_speed_generator_outputs().run_creep_speed, speed_generator.get_inputs().run_profile_1);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_speed_generator_outputs().run_purge_accel, speed_generator.get_inputs().run_profile_2);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_speed_generator_outputs().run_ignition_accel, speed_generator.get_inputs().run_profile_3);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_speed_generator_outputs().run_assist_accel, speed_generator.get_inputs().run_profile_4);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_speed_generator_outputs().hold_speed, speed_generator.get_inputs().hold_speed);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_speed_generator_outputs().decelerate, speed_generator.get_inputs().decelerate);
    // signals_connector::connect_output_to_input(
    //     external_ios.get_speed_generator_outputs().idle, speed_generator.get_inputs().idle);

    // ==================== Speed Controller Inputs ====================
    // Speed reference from startup sequencer (rate-limited)
    signals_connector::connect_output_to_input(
        startup_sequencer.get_outputs().speed_reference, speed_controller.get_inputs().speed_reference);
    // Measured speed from observer
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, speed_controller.get_inputs().measured_speed);

    // ==================== Startup Sequencer → Component Enable Connections ====================
    // Speed controller enable from startup sequencer
    signals_connector::connect_output_to_input(
        startup_sequencer.get_outputs().release_speed_controller, speed_controller.get_inputs().release_pulses);

    // Applied torque reference for bumpless transfer (actual torque from torque controller)
    signals_connector::connect_output_to_input(startup_sequencer.get_outputs().virtual_torque_reference,
        speed_controller.get_inputs().applied_torque_reference);

    // Keep machine_integrator_supervisor for anti-windup freeze logic
    signals_connector::connect_output_to_input(
        machine_integrator_supervisor.get_outputs().freeze_integrator, speed_controller.get_inputs().freeze_integrator);
    signals_connector::connect_output_to_input(
        machine_integrator_supervisor.get_outputs().preset_request, speed_controller.get_inputs().preset_request);

    // Grid-side saturation signal
    signals_connector::connect_output_to_input(
        external_ios.get_fiber_optics_inputs().is_saturated, speed_controller.get_inputs().is_saturated);

    // ==================== Torque Controller Inputs ====================
    signals_connector::connect_output_to_input(
        speed_controller.get_outputs().torque_reference, torque_controller.get_inputs().torque_reference);

    // Mock speed_generator outputs during startup sequencer coordination
    // These will be reconnected when speed_generator is reintegrated
    yeet::mock_signal_input::write(torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{2.0});
    yeet::mock_signal_input::write(torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});
    yeet::mock_signal_input::write(torque_controller.get_inputs().enable_breakaway_boost, false);
    // signals_connector::connect_output_to_input(
    //     speed_generator.get_outputs().torque_limit_max_pu, torque_controller.get_inputs().dynamic_max_torque_limit);
    // signals_connector::connect_output_to_input(
    //     speed_generator.get_outputs().torque_limit_min_pu, torque_controller.get_inputs().dynamic_min_torque_limit);
    // signals_connector::connect_output_to_input(speed_generator.get_outputs().is_breakaway_boost_active,
    //     torque_controller.get_inputs().enable_breakaway_boost);

    // Saturation signal from grid-side
    signals_connector::connect_output_to_input(
        external_ios.get_fiber_optics_inputs().is_saturated, torque_controller.get_inputs().is_saturated);
    // Control mode and manual torque reference from startup sequencer
    signals_connector::connect_output_to_input(
        startup_sequencer.get_outputs().torque_control_mode, torque_controller.get_inputs().control_mode);
    signals_connector::connect_output_to_input(startup_sequencer.get_outputs().virtual_torque_reference,
        torque_controller.get_inputs().manual_torque_reference);

    // ==================== Machine-Side Controller Inputs ====================
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, machine_side_controller.get_inputs().machine_speed);
    signals_connector::connect_output_to_input(
        external_ios.get_fiber_optics_inputs().dc_link_current, machine_side_controller.get_inputs().dc_link_current);

    // ==================== Machine-Side Modulator Inputs ====================
    signals_connector::connect_output_to_input(
        machine_side_controller.get_outputs().firing_angle, machine_side_modulator.get_inputs().firing_angle);
    signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_reference_angle,
        machine_side_modulator.get_inputs().machine_reference_angle);
    signals_connector::connect_output_to_input(external_ios.get_fiber_optics_inputs().force_commutation_acknowledged,
        machine_side_modulator.get_inputs().force_commutation_acknowledged);
    signals_connector::connect_output_to_input(
        external_ios.get_fiber_optics_inputs().operating_mode, machine_side_modulator.get_inputs().operating_mode);
    // Reference frame source and virtual angle from startup sequencer
    signals_connector::connect_output_to_input(startup_sequencer.get_outputs().reference_frame_source,
        machine_side_modulator.get_inputs().reference_frame_source);
    signals_connector::connect_output_to_input(
        startup_sequencer.get_outputs().virtual_rotor_angle, machine_side_modulator.get_inputs().virtual_rotor_angle);
    // Modulator enable from startup sequencer
    signals_connector::connect_output_to_input(startup_sequencer.get_outputs().release_machine_side_pulses,
        machine_side_modulator.get_inputs().release_pulses);

    // ==================== Machine-Side Pulse Generator Inputs ====================
    signals_connector::connect_output_to_input(
        machine_side_modulator.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);
    signals_connector::connect_output_to_input(
        machine_side_modulator.get_outputs().release_pulses, machine_side_pulse_generator.get_inputs().release_pulses);
    signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_reference_angle,
        machine_side_pulse_generator.get_inputs().reference_angle);

    // ==================== External IO Output Connections ====================
    signals_connector::connect_output_to_input(machine_side_pulse_generator.get_outputs().firing_pulses,
        external_ios.get_thyristor_control_output().firing_pulses);
}

}  // namespace yeet::application::sfc::integration
