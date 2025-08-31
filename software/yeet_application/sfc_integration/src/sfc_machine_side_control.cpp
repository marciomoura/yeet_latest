#include "sfc_integration/sfc_machine_side_control.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_control::initialize()
{
    speed_controller.initialize();
    machine_side_controller.initialize();
    machine_side_observer.initialize();
    machine_side_modulator.initialize();
    machine_side_pulse_generator.initialize();
    machine_measurements.initialize();
}

void sfc_machine_side_control::update_10ms()
{
    // 10ms sampling: Pulse generators
    speed_controller.run();
}

void sfc_machine_side_control::update_1ms()
{
    // 1ms sampling: Machine-side controller
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

void sfc_machine_side_control::connect_internal_signals()
{
    // Speed controller inputs
    signals_connector::connect_output_to_input(speed_reference_pu, speed_controller.get_inputs().speed_reference_pu);
    //    signals_connector::connect_output_to_input(
    //        enable_speed_control, speed_controller.get_inputs().enable_speed_control);
    //  signals_connector::connect_output_to_input(freeze_integrator, speed_controller.get_inputs().freeze_integrator);
    // signals_connector::connect_output_to_input(
    //    current_controller_is_saturated, speed_controller.get_inputs().current_controller_is_saturated);

    // Observer feedback to speed controller
    signals_connector::connect_output_to_input(
        machine_side_observer.get_outputs().estimated_speed, speed_controller.get_inputs().measured_speed_pu);

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
}

}  // namespace yeet::application::sfc::integration
