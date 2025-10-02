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

void sfc_grid_side_control::update_10ms() { state_machine.run(); }

void sfc_grid_side_control::update_1ms()
{
    operation_mode_monitor.run();
    current_controller.run();
}

void sfc_grid_side_control::update_200us()
{
    grid_measurements.run();
    dc_link_measurements.run();
    // grid_side_observer.run();
    grid_side_modulator.run();
    grid_side_pulse_generator.run();
}

void sfc_grid_side_control::connect_internal_signals()
{
    // State machine → Components
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().pulse_releases, grid_side_pulse_generator.get_inputs().release_pulses);

    signals_connector::connect_output_to_input(
        state_machine.get_outputs().freeze_integrator, current_controller.get_inputs().freeze_integrator);

    signals_connector::connect_output_to_input(
        state_machine.get_outputs().pulse_releases, operation_mode_monitor.get_inputs().enable);

    signals_connector::connect_output_to_input(
        state_machine.get_outputs().pulse_releases, grid_side_modulator.get_inputs().enable);

    // External state machine inputs
    signals_connector::connect_output_to_input(auxiliaries_are_on, state_machine.get_inputs().auxiliaries_are_on);
    signals_connector::connect_output_to_input(breakers_are_on, state_machine.get_inputs().breakers_are_on);
    signals_connector::connect_output_to_input(start_command, state_machine.get_inputs().start_command);
    signals_connector::connect_output_to_input(stop_command, state_machine.get_inputs().stop_command);
    signals_connector::connect_output_to_input(shutdown_request, state_machine.get_inputs().shutdown_request);
    signals_connector::connect_output_to_input(
        is_current_extinguished, state_machine.get_inputs().is_current_extinguished);

    // Operation mode monitor inputs
    // Note: measured_speed_pu is connected externally from machine-side observer
    signals_connector::connect_output_to_input(
        dc_link_measurements.get_outputs().dc_current_pu, operation_mode_monitor.get_inputs().dc_link_current_pu);

    // Operation mode → Modulator
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().operating_mode, grid_side_modulator.get_inputs().operating_mode);
    signals_connector::connect_output_to_input(
        operation_mode_monitor.get_outputs().is_current_zero, grid_side_modulator.get_inputs().dc_link_current_is_zero);

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

    // Force commutation handshake
    // signals_connector::connect_output_to_input(
    //     request_force_commutation, grid_side_modulator.get_inputs().request_force_commutation);

    // Grid measurements → Observer
    signals_connector::connect_output_to_input(
        grid_measurements.get_outputs().grid_voltage_abc, grid_side_observer.get_inputs().grid_voltage_pu);

    // Observer → Pulse generator synchronization
    signals_connector::connect_output_to_input(
        grid_side_observer.get_outputs().estimated_position, grid_side_pulse_generator.get_inputs().reference_angle);
}

}  // namespace yeet::application::sfc::integration
