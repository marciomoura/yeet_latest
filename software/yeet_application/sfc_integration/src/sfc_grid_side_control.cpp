#include "sfc_integration/sfc_grid_side_control.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_control::initialize()
{
    state_machine.initialize();
    operation_mode_monitor.initialize();
    current_controller.initialize();
    grid_side_observer.initialize();
    grid_side_modulator.initialize();
    grid_side_pulse_generator.initialize();
    grid_measurements.initialize();
    dc_link_measurements.initialize();
}

void sfc_grid_side_control::register_datamodel(yeet::model_registry& registry)
{
    // Register all component parameters with the model_registry
    // This makes them accessible via the parameter configuration protocol

    // Register per-unit system parameters first (fundamental base quantities)
    pu_system_datamodel.register_datamodel(registry, per_unit_system);

    current_controller.register_datamodel(registry);
    grid_side_observer.register_datamodel(registry);
    grid_side_modulator.register_datamodel(registry);
    grid_side_pulse_generator.register_datamodel(registry);
    grid_measurements.register_datamodel(registry);
    dc_link_measurements.register_datamodel(registry);
    state_machine.register_datamodel(registry);
    operation_mode_monitor.register_datamodel(registry);
}

void sfc_grid_side_control::update_10ms() { state_machine.run(); }

void sfc_grid_side_control::update_1ms() { current_controller.run(); }

void sfc_grid_side_control::update_200us()
{
    grid_measurements.run();
    dc_link_measurements.run();
    operation_mode_monitor.run();
    grid_side_observer.run();
    grid_side_modulator.run();
    grid_side_pulse_generator.run();
}

void sfc_grid_side_control::connect_internal_signals()
{
    // NOTE: I/O interface connections are established automatically in sfc_grid_side_control_io constructor
    // through direct reference initialization. External systems can connect to io.analog_inputs, io.digital_inputs,
    // etc. which are references to the actual component signal_input members.

    // ==================== Internal Component Signal Flow ====================

    // State machine → Components
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().pulse_releases, grid_side_pulse_generator.get_inputs().release_pulses);

    signals_connector::connect_output_to_input(
        state_machine.get_outputs().freeze_integrator, current_controller.get_inputs().freeze_integrator);

    signals_connector::connect_output_to_input(
        state_machine.get_outputs().pulse_releases, operation_mode_monitor.get_inputs().enable);

    signals_connector::connect_output_to_input(
        state_machine.get_outputs().pulse_releases, grid_side_modulator.get_inputs().enable);

    signals_connector::connect_output_to_input(state_machine.get_outputs().request_extinguish_dc_current,
        grid_side_modulator.get_inputs().request_extinguish_dc_current);

    // Operation mode monitor inputs
    signals_connector::connect_output_to_input(
        dc_link_measurements.get_outputs().dc_current_pu, operation_mode_monitor.get_inputs().dc_link_current_pu);

    // Operation mode → Modulator
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().operating_mode, grid_side_modulator.get_inputs().operating_mode);
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().is_current_zero, grid_side_modulator.get_inputs().dc_link_current_is_zero);

    // Operation mode → State Machine (Zero-current feedback for shutdown confirmation)
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().is_current_zero, state_machine.get_inputs().is_current_extinguished);

    // Observer → Current controller
    signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_voltage_magnitude,
        current_controller.get_inputs().grid_side_voltage_magnitude);

    // DC link measurements from grid measurements
    signals_connector::connect_output_to_input(
        grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_side_current);

    // DC link measurement → Current controller
    signals_connector::connect_output_to_input(
        dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().dc_link_current_measured);

    // Current controller → Modulator
    signals_connector::connect_output_to_input(
        current_controller.get_outputs().firing_angle, grid_side_modulator.get_inputs().firing_angle);

    // Modulator → Current controller (freeze request)
    signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().is_requesting_main_pi_reset,
        current_controller.get_inputs().modulator_freeze_integrator);

    // Modulator → Pulse generator
    signals_connector::connect_output_to_input(
        grid_side_modulator.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);

    // Grid measurements → Observer
    signals_connector::connect_output_to_input(
        grid_measurements.get_outputs().grid_voltage_abc, grid_side_observer.get_inputs().grid_voltage_pu);

    // Observer → Pulse generator synchronization
    signals_connector::connect_output_to_input(
        grid_side_observer.get_outputs().estimated_position, grid_side_pulse_generator.get_inputs().reference_angle);

    // Internal feedback: Pulse generator → Measurements (for commutation detection)
    signals_connector::connect_output_to_input(
        grid_side_pulse_generator.get_outputs().firing_pulses, grid_measurements.get_inputs().grid_pulses);

    // Observer → Measurements (for sensorless operation - estimated angle/frequency)
    signals_connector::connect_output_to_input(
        grid_side_observer.get_outputs().estimated_position, grid_measurements.get_inputs().grid_side_angle);
    signals_connector::connect_output_to_input(
        grid_side_observer.get_outputs().estimated_frequency, grid_measurements.get_inputs().grid_side_frequency);
}

}  // namespace yeet::application::sfc::integration
