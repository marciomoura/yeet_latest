#include "sfc_integration/sfc_grid_side_control.hpp"

#include "yeet/signal_wiring.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_control::initialize()
{
    state_machine.initialize();
    operation_mode_monitor.initialize();
    grid_integrator_supervisor.initialize();
    current_controller.initialize();
    grid_side_observer.initialize();
    grid_side_modulator.initialize();
    grid_measurements.initialize();
    dc_link_measurements.initialize();
    grid_protections.initialize();

    // Register callbacks for external_ios parameter changes (allows test overrides)
    external_ios.register_callbacks();
}

void sfc_grid_side_control::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Register all component parameters with the parameter_handler
    // This makes them accessible via the parameter configuration protocol

    // Register per-unit system parameters first (fundamental base quantities)
    pu_system_datamodel.register_datamodel(handler, per_unit_system);

    // Register external I/O datamodel (supervisory inputs, fiber-optics, ADC measurements)
    external_ios.register_datamodel(handler);

    grid_integrator_supervisor.register_datamodel(handler);
    current_controller.register_datamodel(handler);
    grid_side_observer.register_datamodel(handler);
    grid_side_modulator.register_datamodel(handler);
    grid_measurements.register_datamodel(handler);
    dc_link_measurements.register_datamodel(handler);
    state_machine.register_datamodel(handler);
    operation_mode_monitor.register_datamodel(handler);
    grid_protections.register_datamodel(handler);
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
}

std::uint8_t sfc_grid_side_control::encode_fiber_optic_tx_to_machine()
{
    // Encode grid-side signals into fiber-optic protocol
    fiber_optic::grid_to_machine_message tx_msg;
    tx_msg.is_forced_commutation = (get_operating_mode().read() == sfc_operating_mode_t::forced_commutation);
    tx_msg.dc_link_current = dc_link_measurements.get_outputs().dc_current_filtered_pu.read();
    tx_msg.force_commutation_acknowledged = grid_side_modulator.get_outputs().force_commutation_acknowledged.read();
    tx_msg.release_pulses = state_machine.get_outputs().release_pulses.read();
    // Include synchronization signals in fiber-optic protocol
    tx_msg.state_id = static_cast<std::uint8_t>(state_machine.get_outputs().current_state_id.read());
    // Include protection signals from grid-side state machine
    tx_msg.grid_release_protections = state_machine.get_outputs().release_grid_protections.read();
    tx_msg.machine_release_protections = state_machine.get_outputs().release_machine_protections.read();
    tx_msg.communication_ok = _last_rx_decode_success;

    _fiber_optic_tx_length = fiber_optic::encode_grid_to_machine(tx_msg, _fiber_optic_tx_payload.data());
    return _fiber_optic_tx_length;
}

bool sfc_grid_side_control::decode_fiber_optic_rx_from_machine(
    const std::uint32_t* payload, std::uint8_t payload_length)
{
    // Decode machine-side fiber-optic message
    auto rx_msg = fiber_optic::decode_machine_to_grid(payload, payload_length);
    if (!rx_msg) {
        _last_rx_decode_success = false;
        return false;
    }

    // Update external_ios with decoded message
    external_ios.set_fiber_optics_rx(*rx_msg);

    _last_rx_decode_success = true;
    return true;
}

void sfc_grid_side_control::connect_internal_signals()
{
    // Type aliases for cleaner member pointer syntax
    using state_machine_inputs = sfc_state_machine_datamodel::inputs;
    using supervisor_inputs = sfc_grid_side_integrator_supervisor_datamodel::inputs;
    using measurements_inputs = sfc_grid_side_measurements_datamodel::inputs;
    using dc_link_inputs = sfc_dc_link_measurements_datamodel::inputs;
    using mode_monitor_inputs = sfc_operation_mode_monitor_datamodel::inputs;
    using protections_inputs = sfc_grid_side_protections_datamodel::inputs;
    using current_ctrl_inputs = sfc_current_controller_datamodel::inputs;
    using observer_inputs = sfc_grid_side_observer_datamodel::inputs;
    using modulator_inputs = sfc_grid_side_modulator_datamodel::inputs;

    // ==================== State Machine Inputs ====================
    wire(state_machine)
        .from(external_ios.get_digital_inputs().auxiliaries_are_on)
        .to(&state_machine_inputs::auxiliaries_are_on)
        .from(external_ios.get_digital_inputs().mv_breaker_is_closed)
        .to(&state_machine_inputs::mv_breaker_is_closed)
        .from(external_ios.get_digital_inputs().start_command)
        .to(&state_machine_inputs::start_command)
        .from(external_ios.get_digital_inputs().stop_command)
        .to(&state_machine_inputs::stop_command)
        .from(operation_mode_monitor.get_outputs().current_is_zero)
        .to(&state_machine_inputs::is_current_extinguished)
        .from(external_ios.get_digital_inputs().mv_breaker_is_tripping)
        .to(&state_machine_inputs::mv_breaker_is_tripping)
        .from(external_ios.get_digital_inputs().external_fault)
        .to(&state_machine_inputs::external_fault)
        .from(external_ios.get_digital_inputs().emergency_off)
        .to(&state_machine_inputs::emergency_off)
        .from(external_ios.get_digital_inputs().door_is_open)
        .to(&state_machine_inputs::door_is_open)
        .from(grid_protections.get_outputs().is_inhibited)
        .to(&state_machine_inputs::protection_inhibited)
        .from(grid_protections.get_outputs().has_active_trip)
        .to(&state_machine_inputs::any_protection_fault_active)
        .from(external_ios.get_fiber_optics_inputs_from_machine().protection_inhibit)
        .to(&state_machine_inputs::machine_side_protection_inhibited)
        .from(external_ios.get_fiber_optics_inputs_from_machine().machine_protection_tripped)
        .to(&state_machine_inputs::machine_side_protection_tripped);

    // ==================== Grid Integrator Supervisor Inputs ====================
    wire(grid_integrator_supervisor)
        .from(grid_side_modulator.get_outputs().freeze_integrator)
        .to(&supervisor_inputs::forced_commutation_freeze_integrator)
        .from(state_machine.get_outputs().release_pulses)
        .to(&supervisor_inputs::state_machine_release_pulses)
        .from(current_controller.get_outputs().preset_complete)
        .to(&supervisor_inputs::preset_complete)
        .from(grid_protections.get_outputs().is_inhibited)
        .to(&supervisor_inputs::grid_protection_shutdown_request);

    // ==================== Grid Measurements Inputs ====================
    wire(grid_measurements)
        .from(external_ios.get_analog_inputs().v_ab_counts)
        .to(&measurements_inputs::v_ab_counts)
        .from(external_ios.get_analog_inputs().v_bc_counts)
        .to(&measurements_inputs::v_bc_counts)
        .from(external_ios.get_analog_inputs().i_a_counts)
        .to(&measurements_inputs::i_a_counts)
        .from(external_ios.get_analog_inputs().i_c_counts)
        .to(&measurements_inputs::i_c_counts)
        .from(grid_side_modulator.get_outputs().firing_pulses)
        .to(&measurements_inputs::grid_pulses)
        .from(grid_side_observer.get_outputs().estimated_reference_angle)
        .to(&measurements_inputs::grid_angle)
        .from(grid_side_observer.get_outputs().estimated_frequency)
        .to(&measurements_inputs::grid_frequency);

    // ==================== DC Link Measurements Inputs ====================
    wire(dc_link_measurements)
        .from(grid_measurements.get_outputs().grid_current_abc)
        .to(&dc_link_inputs::grid_current_abc);

    // ==================== Operation Mode Monitor Inputs ====================
    wire(operation_mode_monitor)
        .from(grid_integrator_supervisor.get_outputs().release_pulses)
        .to(&mode_monitor_inputs::enable)
        .from(dc_link_measurements.get_outputs().dc_current_pu)
        .to(&mode_monitor_inputs::dc_link_current)
        .from(external_ios.get_fiber_optics_inputs_from_machine().measured_speed)
        .to(&mode_monitor_inputs::measured_speed);

    // ==================== Grid Protections Inputs ====================
    wire(grid_protections)
        .from(dc_link_measurements.get_outputs().dc_current_filtered_pu)
        .to(&protections_inputs::dc_link_current_pu)
        .from(grid_side_observer.get_outputs().estimated_voltage_magnitude)
        .to(&protections_inputs::grid_voltage_magnitude_pu)
        .from(state_machine.get_outputs().release_grid_protections)
        .to(&protections_inputs::release_protections)
        .from(external_ios.get_digital_inputs().door_is_open)
        .to(&protections_inputs::door_is_open)
        .from(external_ios.get_digital_inputs().external_fault)
        .to(&protections_inputs::external_fault)
        .from(external_ios.get_digital_inputs().emergency_off)
        .to(&protections_inputs::emergency_off);

    // ==================== Current Controller Inputs ====================
    wire(current_controller)
        .from(grid_integrator_supervisor.get_outputs().freeze_integrator)
        .to(&current_ctrl_inputs::freeze_integrator)
        .from(grid_integrator_supervisor.get_outputs().preset_controller)
        .to(&current_ctrl_inputs::preset_request)
        .from(dc_link_measurements.get_outputs().dc_current_pu)
        .to(&current_ctrl_inputs::current_measured)
        .from(grid_side_observer.get_outputs().estimated_voltage_magnitude)
        .to(&current_ctrl_inputs::grid_voltage_magnitude)
        .from(external_ios.get_fiber_optics_inputs_from_machine().current_reference)
        .to(&current_ctrl_inputs::current_reference)
        .from(external_ios.get_fiber_optics_inputs_from_machine().machine_voltage_magnitude)
        .to(&current_ctrl_inputs::machine_voltage_magnitude);

    // ==================== Grid Side Observer Inputs ====================
    wire(grid_side_observer)
        .from(grid_measurements.get_outputs().grid_voltage_abc)
        .to(&observer_inputs::grid_voltage_abc);

    // ==================== Grid Side Modulator Inputs ====================
    wire(grid_side_modulator)
        .from(state_machine.get_outputs().request_extinguish_dc_current)
        .to(&modulator_inputs::request_extinguish_dc_current)
        .from(grid_integrator_supervisor.get_outputs().shutdown_request)
        .to(&modulator_inputs::shutdown_request)
        .from(grid_protections.get_outputs().is_inhibited)
        .to(&modulator_inputs::protection_trip)
        .from(operation_mode_monitor.get_outputs().operating_mode)
        .to(&modulator_inputs::operating_mode)
        .from(operation_mode_monitor.get_outputs().current_is_zero)
        .to(&modulator_inputs::current_is_zero)
        .from(current_controller.get_outputs().firing_angle)
        .to(&modulator_inputs::firing_angle)
        .from(grid_side_observer.get_outputs().estimated_reference_angle)
        .to(&modulator_inputs::grid_reference_angle)
        .from(external_ios.get_fiber_optics_inputs_from_machine().request_force_commutation)
        .to(&modulator_inputs::request_force_commutation)
        .from(grid_integrator_supervisor.get_outputs().release_pulses)
        .to(&modulator_inputs::release_pulses);

    // ==================== Main Circuit Breaker Outputs ====================
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().mv_breaker_close, external_ios.get_digital_outputs().mv_breaker_close);
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().mv_breaker_open, external_ios.get_digital_outputs().mv_breaker_open);

    // ==================== Auxiliaries Command Output ====================
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().auxiliaries_on_command, external_ios.get_digital_outputs().auxiliaries_on_command);

    // ==================== ACU Fan Outputs ====================
    // All four ACU fans are driven by auxiliaries_on_command
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().auxiliaries_on_command, external_ios.get_digital_outputs().on_command_acu_fan_1);
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().auxiliaries_on_command, external_ios.get_digital_outputs().on_command_acu_fan_2);
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().auxiliaries_on_command, external_ios.get_digital_outputs().on_command_acu_fan_3);
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().auxiliaries_on_command, external_ios.get_digital_outputs().on_command_acu_fan_4);

    // ==================== Software Trip Outputs ====================
    // All three software trip outputs are driven by protection trip status
    signals_connector::connect_output_to_input(
        grid_protections.get_outputs().is_inhibited, external_ios.get_digital_outputs().software_trip_1);
    signals_connector::connect_output_to_input(
        grid_protections.get_outputs().is_inhibited, external_ios.get_digital_outputs().software_trip_2);
    signals_connector::connect_output_to_input(
        grid_protections.get_outputs().is_inhibited, external_ios.get_digital_outputs().software_trip_3);

    // ==================== Emergency Off Reset Output ====================
    signals_connector::connect_output_to_input(
        grid_protections.get_outputs().emergency_off_reset, external_ios.get_digital_outputs().emergency_off_reset);
}

}  // namespace yeet::application::sfc::integration
