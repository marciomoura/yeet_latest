#include "sfc_integration/sfc_grid_side_control.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_control::initialize()
{
    state_machine.initialize();
    operation_mode_monitor.initialize();
    grid_integrator_supervisor.initialize();
    current_controller.initialize();
    grid_side_observer.initialize();
    grid_side_modulator.initialize();
    grid_side_pulse_generator.initialize();
    grid_measurements.initialize();
    dc_link_measurements.initialize();
    grid_protections.initialize();

    // Register callbacks for external_ios parameter changes (allows test overrides)
    external_ios.register_callbacks();
}

void sfc_grid_side_control::register_datamodel(yeet::model_registry& registry)
{
    // Register all component parameters with the model_registry
    // This makes them accessible via the parameter configuration protocol

    // Register per-unit system parameters first (fundamental base quantities)
    pu_system_datamodel.register_datamodel(registry, per_unit_system);

    // Register external I/O datamodel (supervisory inputs, fiber-optics, ADC measurements)
    external_ios.register_datamodel(registry);

    grid_integrator_supervisor.register_datamodel(registry);
    current_controller.register_datamodel(registry);
    grid_side_observer.register_datamodel(registry);
    grid_side_modulator.register_datamodel(registry);
    grid_side_pulse_generator.register_datamodel(registry);
    grid_measurements.register_datamodel(registry);
    dc_link_measurements.register_datamodel(registry);
    state_machine.register_datamodel(registry);
    operation_mode_monitor.register_datamodel(registry);
    grid_protections.register_datamodel(registry);
}

void sfc_grid_side_control::reset() { grid_protections.reset(); }

void sfc_grid_side_control::update_10ms() { state_machine.run(); }

void sfc_grid_side_control::update_1ms()
{
    grid_integrator_supervisor.run();
    current_controller.run();
}

void sfc_grid_side_control::update_200us()
{
    grid_measurements.run();
    dc_link_measurements.run();
    operation_mode_monitor.run();
    grid_protections.run();
    grid_side_observer.run();
    grid_side_modulator.run();
    grid_side_pulse_generator.run();
}

std::uint8_t sfc_grid_side_control::encode_fiber_optic_tx_to_machine()
{
    // Encode grid-side signals into fiber-optic protocol
    fiber_optic::grid_to_machine_message tx_msg;
    tx_msg.is_forced_commutation = (get_operating_mode().read() == sfc_operating_mode_t::forced_commutation);
    tx_msg.dc_link_current = dc_link_measurements.get_outputs().dc_current_filtered_pu.read();
    tx_msg.grid_voltage_magnitude = grid_side_observer.get_outputs().estimated_voltage_magnitude.read();
    tx_msg.force_commutation_acknowledged = grid_side_modulator.get_outputs().force_commutation_acknowledged.read();
    tx_msg.release_pulses = state_machine.get_outputs().release_pulses.read();
    // Include synchronization signals in fiber-optic protocol
    tx_msg.state_id = static_cast<std::uint8_t>(state_machine.get_outputs().current_state_id.read());
    // Include protection signals from grid-side state machine
    tx_msg.grid_release_protections = state_machine.get_outputs().release_protections.read();
    tx_msg.grid_trip_acknowledged = state_machine.get_outputs().trip_acknowledged.read();

    _fiber_optic_tx_length = fiber_optic::encode_grid_to_machine(tx_msg, _fiber_optic_tx_payload.data());
    return _fiber_optic_tx_length;
}

bool sfc_grid_side_control::decode_fiber_optic_rx_from_machine(
    const std::uint32_t* payload, std::uint8_t payload_length)
{
    // Decode machine-side fiber-optic message
    auto rx_msg = fiber_optic::decode_machine_to_grid(payload, payload_length);
    if (!rx_msg) {
        return false;
    }

    // Update external_ios with decoded message
    external_ios.set_fiber_optics_rx(*rx_msg);

    return true;
}

void sfc_grid_side_control::connect_internal_signals()
{
    // NOTE: I/O interface connections are established automatically in sfc_grid_side_control_io constructor
    // through direct reference initialization. External systems can connect to io.analog_inputs, io.digital_inputs,
    // etc. which are references to the actual component signal_input members.

    // ==================== Signal Connections Organized by Receiving Component ====================

    // ==================== State Machine Inputs ====================
    signals_connector::connect_output_to_input(
        external_ios.get_digital_inputs().auxiliaries_are_on, state_machine.get_inputs().auxiliaries_are_on);
    signals_connector::connect_output_to_input(
        external_ios.get_digital_inputs().breakers_are_on, state_machine.get_inputs().breakers_are_on);
    signals_connector::connect_output_to_input(
        external_ios.get_digital_inputs().start_command, state_machine.get_inputs().start_command);
    signals_connector::connect_output_to_input(
        external_ios.get_digital_inputs().stop_command, state_machine.get_inputs().stop_command);
    signals_connector::connect_output_to_input(
        external_ios.get_digital_inputs().shutdown_request, state_machine.get_inputs().shutdown_request);
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().current_is_zero, state_machine.get_inputs().is_current_extinguished);

    // ==================== Grid Integrator Supervisor Inputs ====================
    // Forced commutation freeze from modulator
    signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().freeze_integrator,
        grid_integrator_supervisor.get_inputs().forced_commutation_freeze_integrator);

    // Shutdown request from machine side via fiber optics
    signals_connector::connect_output_to_input(external_ios.get_fiber_optics_inputs_from_machine().shutdown_request,
        grid_integrator_supervisor.get_inputs().machine_side_shutdown_request);

    // Pulse release command from state machine
    signals_connector::connect_output_to_input(state_machine.get_outputs().release_pulses,
        grid_integrator_supervisor.get_inputs().state_machine_release_pulses);

    // Preset complete feedback from current controller
    signals_connector::connect_output_to_input(
        current_controller.get_outputs().preset_complete, grid_integrator_supervisor.get_inputs().preset_complete);

    // ==================== Grid Measurements Inputs ====================
    signals_connector::connect_output_to_input(
        external_ios.get_analog_inputs().v_ab_counts, grid_measurements.get_inputs().v_ab_counts);
    signals_connector::connect_output_to_input(
        external_ios.get_analog_inputs().v_bc_counts, grid_measurements.get_inputs().v_bc_counts);
    signals_connector::connect_output_to_input(
        external_ios.get_analog_inputs().i_a_counts, grid_measurements.get_inputs().i_a_counts);
    signals_connector::connect_output_to_input(
        external_ios.get_analog_inputs().i_c_counts, grid_measurements.get_inputs().i_c_counts);
    signals_connector::connect_output_to_input(
        grid_side_pulse_generator.get_outputs().firing_pulses, grid_measurements.get_inputs().grid_pulses);
    signals_connector::connect_output_to_input(
        grid_side_observer.get_outputs().estimated_reference_angle, grid_measurements.get_inputs().grid_angle);
    signals_connector::connect_output_to_input(
        grid_side_observer.get_outputs().estimated_frequency, grid_measurements.get_inputs().grid_frequency);

    // ==================== DC Link Measurements Inputs ====================
    signals_connector::connect_output_to_input(
        grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_current_abc);

    // ==================== Operation Mode Monitor Inputs ====================
    signals_connector::connect_output_to_input(
        grid_integrator_supervisor.get_outputs().release_pulses, operation_mode_monitor.get_inputs().enable);
    signals_connector::connect_output_to_input(
        dc_link_measurements.get_outputs().dc_current_pu, operation_mode_monitor.get_inputs().dc_link_current);
    signals_connector::connect_output_to_input(external_ios.get_fiber_optics_inputs_from_machine().measured_speed,
        operation_mode_monitor.get_inputs().measured_speed);

    // ==================== Grid Protections Inputs ====================
    signals_connector::connect_output_to_input(
        dc_link_measurements.get_outputs().dc_current_filtered_pu, grid_protections.get_inputs().dc_link_current_pu);
    signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_voltage_magnitude,
        grid_protections.get_inputs().grid_voltage_magnitude_pu);
    // Connect state machine's continuous release signal (active when not in IDLE)
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().release_protections, grid_protections.get_inputs().release_protections);
    // Connect trip acknowledgment from state machine to protection (handshake)
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().trip_acknowledged, grid_protections.get_inputs().trip_acknowledged);

    // Protection shutdown request fed to grid integrator supervisor
    signals_connector::connect_output_to_input(grid_protections.get_outputs().is_tripped,
        grid_integrator_supervisor.get_inputs().grid_protection_shutdown_request);

    // Protection inhibit status fed to state machine (blocks OPERATION_OFF -> OPERATION_ON transition)
    signals_connector::connect_output_to_input(
        grid_protections.get_outputs().is_inhibited, state_machine.get_inputs().protection_inhibited);

    // Protection active fault status (not latched) fed to state machine for trip acknowledgment logic
    signals_connector::connect_output_to_input(
        grid_protections.get_outputs().any_fault_active, state_machine.get_inputs().any_protection_fault_active);

    // Protection trip also connects to state machine for coordinated response
    // Note: State machine shutdown_request input is already connected to external_ios.
    // In a real system, these would be ORed together. For this implementation,
    // protection trips flow through supervisor -> modulator (fast path) and
    // external shutdown comes through external_ios -> state_machine.

    // ==================== Current Controller Inputs ====================
    // Freeze now comes from Grid Integrator Supervisor (unified and ORed)
    signals_connector::connect_output_to_input(
        grid_integrator_supervisor.get_outputs().freeze_integrator, current_controller.get_inputs().freeze_integrator);

    // Preset request from supervisor
    signals_connector::connect_output_to_input(
        grid_integrator_supervisor.get_outputs().preset_controller, current_controller.get_inputs().preset_request);

    signals_connector::connect_output_to_input(
        dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().current_measured);
    signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_voltage_magnitude,
        current_controller.get_inputs().grid_voltage_magnitude);

    signals_connector::connect_output_to_input(external_ios.get_fiber_optics_inputs_from_machine().current_reference,
        current_controller.get_inputs().current_reference);
    signals_connector::connect_output_to_input(
        external_ios.get_fiber_optics_inputs_from_machine().machine_voltage_magnitude,
        current_controller.get_inputs().machine_voltage_magnitude);

    // ==================== Grid Side Observer Inputs ====================
    signals_connector::connect_output_to_input(
        grid_measurements.get_outputs().grid_voltage_abc, grid_side_observer.get_inputs().grid_voltage_abc);

    // ==================== Grid Side Modulator Inputs ====================
    signals_connector::connect_output_to_input(state_machine.get_outputs().request_extinguish_dc_current,
        grid_side_modulator.get_inputs().request_extinguish_dc_current);
    signals_connector::connect_output_to_input(
        grid_integrator_supervisor.get_outputs().shutdown_request, grid_side_modulator.get_inputs().shutdown_request);
    signals_connector::connect_output_to_input(
        grid_protections.get_outputs().is_tripped, grid_side_modulator.get_inputs().protection_trip);
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().operating_mode, grid_side_modulator.get_inputs().operating_mode);
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().current_is_zero, grid_side_modulator.get_inputs().current_is_zero);
    signals_connector::connect_output_to_input(
        current_controller.get_outputs().firing_angle, grid_side_modulator.get_inputs().firing_angle);
    signals_connector::connect_output_to_input(
        external_ios.get_fiber_optics_inputs_from_machine().request_force_commutation,
        grid_side_modulator.get_inputs().request_force_commutation);
    signals_connector::connect_output_to_input(
        grid_integrator_supervisor.get_outputs().release_pulses, grid_side_modulator.get_inputs().release_pulses);

    // ==================== Grid Side Pulse Generator Inputs ====================
    // Pulse releases now come from Grid Integrator Supervisor
    // (supervisor gates state machine pulse_releases based on frozen/tripped status)
    signals_connector::connect_output_to_input(
        grid_integrator_supervisor.get_outputs().release_pulses, grid_side_pulse_generator.get_inputs().release_pulses);

    signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_reference_angle,
        grid_side_pulse_generator.get_inputs().reference_angle);
    signals_connector::connect_output_to_input(
        grid_side_modulator.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);
}

}  // namespace yeet::application::sfc::integration
