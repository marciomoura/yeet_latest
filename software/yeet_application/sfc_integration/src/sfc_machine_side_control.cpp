#include "sfc_integration/sfc_machine_side_control.hpp"

namespace yeet::application::sfc::integration {

// ==================== Internal Bridge Signals ====================
// These are internal-only signals used for routing commands from the IO interface to internal components.
// They exist in the implementation file to avoid cluttering the public class interface.
// External code should use get_io() interface instead.
struct sfc_machine_side_internal_signals {
    // Excitation control bridge signals
    yeet::signal_output<bool> exciter_ready;
    yeet::signal_output<bool> excitation_start_command;
    yeet::signal_output<bool> excitation_stop_command;

    // Speed generator profile selection bridge signals (turbine starting sequence)
    yeet::signal_output<bool> run_creep_speed;     // Profile 1: Creep speed (~5% rated)
    yeet::signal_output<bool> run_purge_accel;     // Profile 2: Purge acceleration (~20% rated)
    yeet::signal_output<bool> run_ignition_accel;  // Profile 3: Ignition acceleration (~55% rated)
    yeet::signal_output<bool> run_assist_accel;    // Profile 4: Assist to self-sustaining (~95% rated)
    yeet::signal_output<bool> hold_speed;          // Hold current speed setpoint
    yeet::signal_output<bool> decelerate;          // Controlled deceleration to zero
    yeet::signal_output<bool> idle;                // Idle mode (zero torque/speed outputs)

    // Speed controller cascade bridge signals from grid-side
    yeet::signal_output<bool> enable_speed_control;         // From grid-side pulse_releases
    yeet::signal_output<bool> freeze_integrator;            // From grid-side state machine
    yeet::signal_output<bool> modulator_freeze_integrator;  // From grid-side modulator PI reset
};

// Global thread-local instance for signal routing within connect_internal_signals
thread_local sfc_machine_side_internal_signals g_machine_side_internal_signals;

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
    // ==================== Excitation Controller Connections ====================

    // Excitation controller inputs from external signals via bridge signals
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.exciter_ready, excitation_controller.get_inputs().exciter_ready);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.excitation_start_command, excitation_controller.get_inputs().start_command);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.excitation_stop_command, excitation_controller.get_inputs().stop_command);

    // Excitation controller feedback: machine speed from observer
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, excitation_controller.get_inputs().machine_rotor_speed_pu);

    // ==================== Speed Controller Connections ====================

    // Speed controller inputs
    // Note: speed_reference_pu is now sourced exclusively from speed_generator (below)
    // For manual testing without speed_generator, connect external speed_reference_pu signal here

    // Observer feedback to speed controller
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, speed_controller.get_inputs().measured_speed_pu);

    // Speed controller cascade inputs from grid-side (external connections via bridge signals)
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.enable_speed_control, speed_controller.get_inputs().enable_speed_control);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.freeze_integrator, speed_controller.get_inputs().freeze_integrator);
    signals_connector::connect_output_to_input(g_machine_side_internal_signals.modulator_freeze_integrator,
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
        g_machine_side_internal_signals.run_creep_speed, speed_generator.get_inputs().run_profile_1);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.run_purge_accel, speed_generator.get_inputs().run_profile_2);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.run_ignition_accel, speed_generator.get_inputs().run_profile_3);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.run_assist_accel, speed_generator.get_inputs().run_profile_4);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.hold_speed, speed_generator.get_inputs().hold_speed);
    signals_connector::connect_output_to_input(
        g_machine_side_internal_signals.decelerate, speed_generator.get_inputs().decelerate);
    signals_connector::connect_output_to_input(g_machine_side_internal_signals.idle, speed_generator.get_inputs().idle);

    // ==================== Torque Controller Connections ====================

    // Speed Controller → Torque Controller (cascade)
    signals_connector::connect_output_to_input(speed_controller.get_outputs().torque_reference,
        torque_controller.get_inputs().torque_reference_from_speed_controller);

    // Note: Dynamic torque limits and boost enable are connected directly from speed generator above (lines 80-86)
    // Note: Current controller saturation feedback is connected externally (cross-controller)

    // ==================== Machine-Side Controller Connections ====================

    // Machine-side controller inputs
    // signals_connector::connect_output_to_input(
    // dc_link_current_pu, machine_side_controller.get_inputs().dc_link_current);
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, machine_side_controller.get_inputs().machine_speed);

    // Machine-side modulator inputs
    signals_connector::connect_output_to_input(
        machine_side_controller.get_outputs().firing_angle, machine_side_modulator.get_inputs().firing_angle);
    // Note: enable signal is connected externally from grid-side state machine
    // signals_connector::connect_output_to_input(operating_mode, machine_side_modulator.get_inputs().operating_mode);
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_position, machine_side_modulator.get_inputs().machine_side_angle);
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

    // Observer → Measurements (for sensorless operation - estimated angle/frequency)
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_position, machine_measurements.get_inputs().machine_side_angle);
    signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_frequency,
        machine_measurements.get_inputs().machine_side_frequency);
}

}  // namespace yeet::application::sfc::integration
