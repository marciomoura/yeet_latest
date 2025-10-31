#include "sfc_integration/sfc_machine_side_control.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_control::initialize()
{
    excitation_controller.initialize();
    speed_generator.initialize();
    speed_controller.initialize();
    torque_controller.initialize();
    machine_side_controller.initialize();
    machine_side_observer.initialize();
    machine_side_modulator.initialize();
    machine_side_pulse_generator.initialize();
    machine_measurements.initialize();
}

void sfc_machine_side_control::update_10ms()
{
    // Apply external input overrides from datamodel (reads override flags and writes to outputs)
    // external_ios.update(excitation_controller.get_outputs().enable_excitation.read() ||
    //                         external_ios.get_parameters().exciter_ready.read(),
    //     external_ios.get_parameters().excitation_start_command.read(),
    //     external_ios.get_parameters().excitation_stop_command.read(),
    //     external_ios.get_parameters().run_creep_speed.read(), external_ios.get_parameters().run_purge_accel.read(),
    //     external_ios.get_parameters().run_ignition_accel.read(),
    //     external_ios.get_parameters().run_assist_accel.read(), external_ios.get_parameters().hold_speed.read(),
    //     external_ios.get_parameters().decelerate.read(), external_ios.get_parameters().idle.read(),
    //     external_ios.get_parameters().enable_speed_control.read(),
    //     external_ios.get_parameters().freeze_integrator.read(),
    //     external_ios.get_parameters().modulator_freeze_integrator.read());

    // 10ms sampling: Execute in order: generator → controller → limiter
    speed_generator.run();    // 1. Generate speed reference + dynamic torque limits
    speed_controller.run();   // 2. Process speed error → torque reference
    torque_controller.run();  // 3. Apply limits + boost → current reference
}

void sfc_machine_side_control::update_1ms()
{
    // 1ms sampling: Excitation controller, machine-side controller
    excitation_controller.run();
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

void sfc_machine_side_control::register_datamodel(yeet::model_registry& registry)
{
    // Register all component parameters with the model_registry
    // This makes them accessible via the parameter configuration protocol

    // Register per-unit system parameters first (fundamental base quantities)
    pu_system_datamodel.register_datamodel(registry, per_unit_system);
    // Register external I/O overrides datamodel
    external_ios.register_datamodel(registry);

    excitation_controller.register_datamodel(registry);
    speed_generator.register_datamodel(registry);
    speed_controller.register_datamodel(registry);
    torque_controller.register_datamodel(registry);
    machine_side_controller.register_datamodel(registry);
    machine_side_observer.register_datamodel(registry);
    machine_side_modulator.register_datamodel(registry);
    machine_side_pulse_generator.register_datamodel(registry);
    machine_measurements.register_datamodel(registry);
}

void sfc_machine_side_control::connect_internal_signals()
{
    signals_connector::connect_output_to_input(external_ios.get_control_inputs().current_controller_is_saturated,
        torque_controller.get_inputs().current_controller_is_saturated);
    signals_connector::connect_output_to_input(external_ios.get_control_inputs().force_commutation_acknowledged,
        machine_side_modulator.get_inputs().force_commutation_acknowledged);
    signals_connector::connect_output_to_input(
        external_ios.get_control_inputs().operating_mode, machine_side_modulator.get_inputs().operating_mode);
    signals_connector::connect_output_to_input(
        external_ios.get_control_inputs().dc_link_current, machine_side_controller.get_inputs().dc_link_current);

    // ==================== Excitation Controller Connections ====================

    // Excitation controller inputs from external signals via bridge signals
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().exciter_ready, excitation_controller.get_inputs().exciter_ready);
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().excitation_start_command, excitation_controller.get_inputs().start_command);
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().excitation_stop_command, excitation_controller.get_inputs().stop_command);

    // Excitation controller feedback: machine speed from observer
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, excitation_controller.get_inputs().machine_rotor_speed_pu);

    // Machine measurements analog inputs
    signals_connector::connect_output_to_input(
        external_ios.get_control_inputs().v_ab_counts, machine_measurements.get_inputs().v_ab_counts);
    signals_connector::connect_output_to_input(
        external_ios.get_control_inputs().v_bc_counts, machine_measurements.get_inputs().v_bc_counts);

    // ==================== Speed Controller Connections ====================

    // Speed controller inputs
    // Note: speed_reference_pu is now sourced exclusively from speed_generator (below)
    // For manual testing without speed_generator, connect external speed_reference_pu signal here

    // Observer feedback to speed controller
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, speed_controller.get_inputs().measured_speed_pu);

    // Speed controller cascade inputs from grid-side (external connections via bridge signals)
    signals_connector::connect_output_to_input(
        external_ios.get_control_inputs().pulse_releases, speed_controller.get_inputs().enable_speed_control);

    signals_connector::connect_output_to_input(external_ios.get_control_inputs().current_controller_is_saturated,
        speed_controller.get_inputs().current_controller_is_saturated);

    // Freeze integrator when force commutation is requested
    // WHEN DO WE FREEZE THE SPEED CONTROLLER ???
    // WHEN TO FREEZE THE INTEGRATOR AND WHEN TO FREEZE THE MODULATOR?
    signals_connector::connect_output_to_input(external_ios.get_control_inputs().current_controller_is_saturated,
        speed_controller.get_inputs().freeze_integrator);
    signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().request_force_commutation,
        speed_controller.get_inputs().modulator_freeze_integrator);

    // ==================== Speed Generator Connections ====================

    // Speed Generator → Speed Controller (speed reference - overrides external signal)
    signals_connector::connect_output_to_input(
        speed_generator.get_outputs().speed_reference_pu, speed_controller.get_inputs().speed_reference_pu);

    // Speed Generator → Torque Controller (dynamic limits + boost enable via intermediate signals)
    signals_connector::connect_output_to_input(
        speed_generator.get_outputs().torque_limit_max_pu, torque_controller.get_inputs().dynamic_max_torque_limit);
    signals_connector::connect_output_to_input(
        speed_generator.get_outputs().torque_limit_min_pu, torque_controller.get_inputs().dynamic_min_torque_limit);
    signals_connector::connect_output_to_input(
        speed_generator.get_outputs().is_breakaway_boost_active, torque_controller.get_inputs().enable_breakaway_boost);

    // Observer → Speed Generator (speed feedback for trajectory tracking)
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, speed_generator.get_inputs().measured_speed_pu);

    // External Commands → Speed Generator (profile selection and control via bridge signals)
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().run_creep_speed, speed_generator.get_inputs().run_profile_1);
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().run_purge_accel, speed_generator.get_inputs().run_profile_2);
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().run_ignition_accel, speed_generator.get_inputs().run_profile_3);
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().run_assist_accel, speed_generator.get_inputs().run_profile_4);
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().hold_speed, speed_generator.get_inputs().hold_speed);
    signals_connector::connect_output_to_input(
        external_ios.get_outputs().decelerate, speed_generator.get_inputs().decelerate);
    signals_connector::connect_output_to_input(external_ios.get_outputs().idle, speed_generator.get_inputs().idle);

    // ==================== Torque Controller Connections ====================

    // Speed Controller → Torque Controller (cascade)
    signals_connector::connect_output_to_input(speed_controller.get_outputs().torque_reference,
        torque_controller.get_inputs().torque_reference_from_speed_controller);

    // Note: Dynamic torque limits and boost enable are connected directly from speed generator above (lines 80-86)
    // Note: Current controller saturation feedback is connected externally (cross-controller)

    // ==================== Machine-Side Controller Connections ====================

    // Machine-side controller inputs
    // signals_connector::connect_output_to_input(
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, machine_side_controller.get_inputs().machine_speed);

    // Machine-side modulator inputs
    signals_connector::connect_output_to_input(
        external_ios.get_control_inputs().pulse_releases, machine_side_modulator.get_inputs().enable);

    signals_connector::connect_output_to_input(
        machine_side_controller.get_outputs().firing_angle, machine_side_modulator.get_inputs().firing_angle);
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_position, machine_side_modulator.get_inputs().machine_side_angle);
    // Note: enable signal is connected externally from grid-side state machine
    // dc_link_current_pu, machine_side_controller.get_inputs().dc_link_current);
    // signals_connector::connect_output_to_input(operating_mode, machine_side_modulator.get_inputs().operating_mode);
    // signals_connector::connect_output_to_input(
    //     force_commutation_acknowledged, machine_side_modulator.get_inputs().force_commutation_acknowledged);

    // Modulator → Pulse generator
    signals_connector::connect_output_to_input(
        machine_side_modulator.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);
    signals_connector::connect_output_to_input(
        machine_side_modulator.get_outputs().release_pulses, machine_side_pulse_generator.get_inputs().release_pulses);

    // Pulse generator synchronization from observer
    signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_position,
        machine_side_pulse_generator.get_inputs().reference_angle);

    // Machine measurements → Observer
    signals_connector::connect_output_to_input(
        machine_measurements.get_outputs().machine_voltage_abc, machine_side_observer.get_inputs().machine_voltage_pu);

    // External machine voltage magnitude feedforward
    // signals_connector::connect_output_to_input(
    //     machine_side_voltage_magnitude, machine_side_observer.get_outputs().estimated_voltage_magnitude);

    // Internal feedback: Pulse generator → Measurements (for commutation detection)
    signals_connector::connect_output_to_input(
        machine_side_pulse_generator.get_outputs().firing_pulses, machine_measurements.get_inputs().machine_pulses);

    // Internal feedback: Pulse generator → External IO
    signals_connector::connect_output_to_input(
        machine_side_pulse_generator.get_outputs().firing_pulses, external_ios.get_control_outputs().firing_pulses);

    // Observer → Measurements (for sensorless operation - estimated angle/frequency)
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_position, machine_measurements.get_inputs().machine_side_angle);
    signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_frequency,
        machine_measurements.get_inputs().machine_side_frequency);
}

}  // namespace yeet::application::sfc::integration
