#include "sfc_integration/sfc_machine_side_control.hpp"

#include "yeet/signal_wiring.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_control::initialize()
{
    excitation_controller.initialize();
    // speed_generator.initialize();
    speed_controller.initialize();
    torque_controller.initialize();
    startup_sequencer.initialize();
    machine_integrator_supervisor.initialize();
    machine_side_controller.initialize();
    machine_side_observer.initialize();
    machine_side_modulator.initialize();
    machine_side_pulse_generator.initialize();
    machine_measurements.initialize();
    machine_protections.initialize();
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
    // 200µs sampling: Measurements, observers, modulators, protections
    machine_measurements.run();
    machine_side_observer.run();
    machine_side_modulator.run();
    machine_side_pulse_generator.run();
    machine_protections.run();
}

std::uint8_t sfc_machine_side_control::encode_fiber_optic_tx_to_grid()
{
    fiber_optic::machine_to_grid_message tx_msg;
    tx_msg.measured_speed = machine_side_observer.get_outputs().estimated_speed.read();
    tx_msg.dc_link_current_reference = torque_controller.get_outputs().current_reference.read();
    tx_msg.machine_voltage_magnitude = machine_measurements.get_outputs().machine_voltage_magnitude.read();
    tx_msg.force_commutation_request = machine_side_modulator.get_outputs().request_force_commutation.read();
    tx_msg.shutdown_request = machine_integrator_supervisor.get_outputs().request_shutdown.read();
    tx_msg.machine_protection_tripped = machine_protections.get_outputs().is_tripped.read();
    tx_msg.protection_inhibit = machine_protections.get_outputs().is_inhibited.read();
    // Machine-side doesn't have a state machine with trip_acknowledged yet
    // This could be added in future if machine-side needs independent acknowledgment logic
    tx_msg.machine_trip_acknowledged = false;
    _fiber_optic_tx_length = fiber_optic::encode_machine_to_grid(tx_msg, _fiber_optic_tx_payload.data());
    return _fiber_optic_tx_length;
}

bool sfc_machine_side_control::decode_fiber_optic_rx_from_grid(
    const std::uint32_t* payload, std::uint8_t payload_length)
{
    auto rx_msg = fiber_optic::decode_grid_to_machine(payload, payload_length);
    if (!rx_msg) return false;

    // Pass the decoded message directly to external_ios - no manual unpacking needed
    external_ios.set_fiber_optics_rx(*rx_msg);
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
    // speed_generator.register_datamodel(registry);
    speed_controller.register_datamodel(registry);
    torque_controller.register_datamodel(registry);
    startup_sequencer.register_datamodel(registry);
    machine_side_controller.register_datamodel(registry);
    machine_side_observer.register_datamodel(registry);
    machine_side_modulator.register_datamodel(registry);
    machine_side_pulse_generator.register_datamodel(registry);
    machine_measurements.register_datamodel(registry);
    machine_protections.register_datamodel(registry);
}

void sfc_machine_side_control::connect_internal_signals()
{
    using namespace yeet;

    // Type aliases for input struct types
    using measurements_inputs = sfc_machine_side_measurements_datamodel::inputs;
    using observer_inputs = sfc_machine_side_observer_datamodel::inputs;
    using supervisor_inputs = sfc_machine_side_integrator_supervisor_datamodel::inputs;
    using excitation_inputs = sfc_excitation_controller_datamodel::inputs;
    using sequencer_inputs = sfc_startup_sequencer_datamodel::inputs;
    using speed_ctrl_inputs = sfc_speed_controller_datamodel::inputs;
    using torque_ctrl_inputs = sfc_torque_controller_datamodel::inputs;
    using controller_inputs = sfc_machine_side_controller_datamodel::inputs;
    using modulator_inputs = sfc_machine_side_modulator_datamodel::inputs;
    using pulse_gen_inputs = sfc_machine_side_pulse_generator_datamodel::inputs;
    using protections_inputs = sfc_machine_side_protections_datamodel::inputs;

    // ==================== Machine Measurements Inputs ====================
    wire(machine_measurements)
        .from(external_ios.get_analog_inputs().v_ab_counts)
        .to(&measurements_inputs::v_ab_counts)
        .from(external_ios.get_analog_inputs().v_bc_counts)
        .to(&measurements_inputs::v_bc_counts)
        .from(machine_side_pulse_generator.get_outputs().firing_pulses)
        .to(&measurements_inputs::machine_pulses)
        .from(machine_side_observer.get_outputs().estimated_reference_angle)
        .to(&measurements_inputs::machine_reference_angle)
        .from(machine_side_observer.get_outputs().estimated_frequency)
        .to(&measurements_inputs::machine_frequency);

    // ==================== Machine-Side Observer Inputs ====================
    wire(machine_side_observer)
        .from(machine_measurements.get_outputs().machine_voltage_abc)
        .to(&observer_inputs::machine_voltage_pu)
        .from(startup_sequencer.get_outputs().release_machine_observer)
        .to(&observer_inputs::release_observer)
        .from(startup_sequencer.get_outputs().preset_observer)
        .to(&observer_inputs::preset_observer)
        .from(startup_sequencer.get_outputs().preset_angle)
        .to(&observer_inputs::preset_angle)
        .from(startup_sequencer.get_outputs().preset_frequency)
        .to(&observer_inputs::preset_frequency);

    // ==================== Machine Integrator Supervisor Inputs ====================
    wire(machine_integrator_supervisor)
        .from(external_ios.get_fiber_optics_inputs_from_grid().release_pulses)
        .to(&supervisor_inputs::grid_side_release_pulses)
        .from(startup_sequencer.get_outputs().release_speed_controller)
        .to(&supervisor_inputs::release_speed_controller)
        .from(torque_controller.get_outputs().is_saturated)
        .to(&supervisor_inputs::torque_saturated)
        .from(torque_controller.get_outputs().is_braking_requested)
        .to(&supervisor_inputs::is_braking_requested)
        .from(machine_protections.get_outputs().is_tripped)
        .to(&supervisor_inputs::protection_shutdown_request);

    // ==================== Excitation Controller Inputs ====================
    wire(excitation_controller)
        .from(machine_side_observer.get_outputs().estimated_speed)
        .to(&excitation_inputs::machine_rotor_speed_pu)
        .from(machine_side_observer.get_outputs().estimated_voltage_magnitude)
        .to(&excitation_inputs::machine_voltage_magnitude_pu)
        .from(startup_sequencer.get_outputs().release_excitation)
        .to(&excitation_inputs::release_excitation);

    // ==================== Startup Sequencer Inputs ====================
    wire(startup_sequencer)
        .from(excitation_controller.get_outputs().is_field_established)
        .to(&sequencer_inputs::is_field_established)
        .from(machine_side_observer.get_outputs().estimated_reference_angle)
        .to(&sequencer_inputs::estimated_rotor_angle)
        .from(machine_side_observer.get_outputs().estimated_speed)
        .to(&sequencer_inputs::measured_speed)
        .from(machine_side_observer.get_outputs().estimated_voltage_magnitude)
        .to(&sequencer_inputs::machine_voltage_magnitude)
        .from(external_ios.get_fiber_optics_inputs_from_grid().current_state_id)
        .to(&sequencer_inputs::current_state_id);

    // ==================== Speed Controller Inputs ====================
    wire(speed_controller)
        .from(startup_sequencer.get_outputs().speed_reference)
        .to(&speed_ctrl_inputs::speed_reference)
        .from(machine_side_observer.get_outputs().estimated_speed)
        .to(&speed_ctrl_inputs::measured_speed)
        .from(startup_sequencer.get_outputs().release_speed_controller)
        .to(&speed_ctrl_inputs::release_pulses)
        .from(torque_controller.get_outputs().torque_reference)
        .to(&speed_ctrl_inputs::applied_torque_reference)
        .from(machine_integrator_supervisor.get_outputs().freeze_integrator)
        .to(&speed_ctrl_inputs::freeze_integrator)
        .from(machine_integrator_supervisor.get_outputs().preset_request)
        .to(&speed_ctrl_inputs::preset_request)
        .from(torque_controller.get_outputs().is_saturated)
        .to(&speed_ctrl_inputs::is_saturated);

    // ==================== Torque Controller Inputs ====================
    wire(torque_controller)
        .from(speed_controller.get_outputs().torque_reference)
        .to(&torque_ctrl_inputs::torque_reference)
        .from(startup_sequencer.get_outputs().speed_reference)
        .to(&torque_ctrl_inputs::speed_reference)
        .from(external_ios.get_fiber_optics_inputs_from_grid().current_state_id)
        .to(&torque_ctrl_inputs::current_state_id)
        .from(machine_protections.get_outputs().is_tripped)
        .to(&torque_ctrl_inputs::machine_protection_tripped)
        .from(startup_sequencer.get_outputs().torque_control_mode)
        .to(&torque_ctrl_inputs::control_mode)
        .from(startup_sequencer.get_outputs().virtual_torque_reference)
        .to(&torque_ctrl_inputs::manual_torque_reference);

    // ==================== Machine-Side Controller Inputs ====================
    wire(machine_side_controller)
        .from(machine_side_observer.get_outputs().estimated_speed)
        .to(&controller_inputs::machine_speed)
        .from(external_ios.get_fiber_optics_inputs_from_grid().dc_link_current)
        .to(&controller_inputs::dc_link_current);

    // ==================== Machine-Side Modulator Inputs ====================
    wire(machine_side_modulator)
        .from(machine_side_controller.get_outputs().firing_angle)
        .to(&modulator_inputs::firing_angle)
        .from(machine_side_observer.get_outputs().estimated_reference_angle)
        .to(&modulator_inputs::machine_reference_angle)
        .from(external_ios.get_fiber_optics_inputs_from_grid().force_commutation_acknowledged)
        .to(&modulator_inputs::force_commutation_acknowledged)
        .from(external_ios.get_fiber_optics_inputs_from_grid().operating_mode)
        .to(&modulator_inputs::operating_mode)
        .from(startup_sequencer.get_outputs().reference_frame_source)
        .to(&modulator_inputs::reference_frame_source)
        .from(startup_sequencer.get_outputs().virtual_rotor_angle)
        .to(&modulator_inputs::virtual_rotor_angle)
        .from(startup_sequencer.get_outputs().release_machine_side_pulses)
        .to(&modulator_inputs::release_pulses);

    // ==================== Machine-Side Pulse Generator Inputs ====================
    wire(machine_side_pulse_generator)
        .from(machine_side_modulator.get_outputs().firing_angle)
        .to(&pulse_gen_inputs::firing_angle)
        .from(machine_side_modulator.get_outputs().release_pulses)
        .to(&pulse_gen_inputs::release_pulses)
        .from(machine_side_observer.get_outputs().estimated_reference_angle)
        .to(&pulse_gen_inputs::reference_angle);

    // ==================== Machine-Side Protections Inputs ====================
    wire(machine_protections)
        .from(external_ios.get_fiber_optics_inputs_from_grid().dc_link_current)
        .to(&protections_inputs::dc_link_current_pu)
        .from(machine_measurements.get_outputs().machine_voltage_magnitude)
        .to(&protections_inputs::machine_voltage_magnitude_pu)
        .from(machine_side_observer.get_outputs().estimated_frequency)
        .to(&protections_inputs::machine_frequency)
        .from(machine_side_observer.get_outputs().estimated_frequency_pu)
        .to(&protections_inputs::machine_frequency_pu)
        .from(machine_side_observer.get_outputs().estimated_speed)
        .to(&protections_inputs::machine_speed)
        .from(external_ios.get_fiber_optics_inputs_from_grid().grid_release_protections)
        .to(&protections_inputs::release_protections)
        .from(external_ios.get_fiber_optics_inputs_from_grid().grid_trip_acknowledged)
        .to(&protections_inputs::trip_acknowledged)
        .from(startup_sequencer.get_outputs().startup_active)
        .to(&protections_inputs::startup_active)
        .from(startup_sequencer.get_outputs().is_startup_complete)
        .to(&protections_inputs::closed_loop_active);

    // ==================== External IO Output Connections ====================
    signals_connector::connect_output_to_input(machine_side_pulse_generator.get_outputs().firing_pulses,
        external_ios.get_thyristor_control_output().firing_pulses);

    // Excitation command outputs from excitation controller to external IOs
    // Map excitation_start_command to excitation_release (DO)
    signals_connector::connect_output_to_input(excitation_controller.get_outputs().excitation_start_command,
        external_ios.get_excitation_outputs().excitation_release);
    // Map field_current_reference to excitation_current_reference (AO)
    signals_connector::connect_output_to_input(excitation_controller.get_outputs().field_current_reference,
        external_ios.get_excitation_outputs().excitation_current_reference);

    // Mock speed_generator outputs during startup sequencer coordination
    // These will be reconnected when speed_generator is reintegrated
    static bool false_storage = false;
    wire(torque_controller)
        .mocked(false_storage)
        .to(&torque_ctrl_inputs::enable_breakaway_boost)
        .mocked(false_storage)
        .to(&torque_ctrl_inputs::is_saturated);
}

}  // namespace yeet::application::sfc::integration
