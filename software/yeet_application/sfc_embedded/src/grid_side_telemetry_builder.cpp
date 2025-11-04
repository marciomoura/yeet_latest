/**
 * @file grid_side_telemetry_builder.cpp
 * @brief Implementation of grid-side telemetry packet builder
 */

#include "sfc_embedded/grid_side_network_telemetry_builder.hpp"

namespace yeet::application::sfc::telemetry {

yeet::telemetry::telemetry_packet grid_side_telemetry_builder::build_packet(std::uint32_t timestamp_ms) noexcept
{
    yeet::telemetry::telemetry_packet packet;

    // Extract control system components
    const auto& grid_meas = _control.get_grid_measurements();
    const auto& dc_meas = _control.get_dc_link_measurements();
    const auto& observer = _control.get_grid_side_observer();
    const auto& modulator = _control.get_grid_side_modulator();
    const auto& current_controller = _control.get_current_controller();
    const auto& mode_monitor = _control.get_operation_mode_monitor();
    const auto& state_machine = _control.get_state_machine();
    const auto& pulse_generator = _control.get_grid_side_pulse_generator();

    // === Grid Measurements Group ===
    // ABC frame measurements
    packet.add_signal(0, static_cast<float>(grid_meas.get_outputs().grid_voltage_magnitude_filtered.read()));
    packet.add_signal(1, static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().a()));
    packet.add_signal(2, static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().b()));
    packet.add_signal(3, static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().c()));

    const auto& grid_current = grid_meas.get_outputs().grid_current_abc.read();
    packet.add_signal(4, static_cast<float>(grid_current.to_alphabeta().magnitude()));  // Grid current magnitude
    packet.add_signal(5, static_cast<float>(grid_current.a()));
    packet.add_signal(6, static_cast<float>(grid_current.b()));
    packet.add_signal(7, static_cast<float>(grid_current.c()));
    packet.add_signal(8, static_cast<float>(dc_meas.get_outputs().dc_current_filtered_pu.read()));

    // === Observer Group ===
    packet.add_signal(14, static_cast<float>(observer.get_outputs().estimated_voltage_magnitude.read()));
    packet.add_signal(15, static_cast<float>(observer.get_outputs().estimated_frequency_pu.read()));
    packet.add_signal(16, static_cast<float>(observer.get_outputs().estimated_position.read().get_pu()));

    // === Modulator Group ===
    packet.add_signal(20, static_cast<float>(modulator.get_inputs().firing_angle.read().get_pu()));
    packet.add_signal(21, static_cast<float>(modulator.get_outputs().firing_angle.read().get_pu()));
    packet.add_signal(22, static_cast<float>(modulator.get_inputs().operating_mode.read()));
    packet.add_signal(23, static_cast<float>(modulator.get_inputs().request_extinguish_dc_current.read()));
    packet.add_signal(24, static_cast<float>(modulator.get_inputs().request_force_commutation.read()));
    packet.add_signal(28, static_cast<float>(modulator.get_outputs().modulator_state.read()));
    packet.add_signal(25, modulator.get_outputs().release_pulses.read() ? 1.0f : 0.0f);  // Modulator outputs
    packet.add_signal(26, modulator.get_outputs().force_commutation_acknowledged.read() ? 1.0f : 0.0f);
    packet.add_signal(27, modulator.get_outputs().freeze_integrator.read() ? 1.0f : 0.0f);

    // === current_controller Group ===
    packet.add_signal(29, static_cast<float>(current_controller.get_inputs().current_reference.read()));
    packet.add_signal(30, static_cast<float>(current_controller.get_inputs().current_measured.read()));
    packet.add_signal(33, static_cast<float>(current_controller.get_outputs().current_reference.read()));
    packet.add_signal(31, static_cast<float>(current_controller.get_inputs().freeze_integrator.read()));
    packet.add_signal(32, static_cast<float>(current_controller.get_inputs().preset_request.read()));
    packet.add_signal(33, static_cast<float>(current_controller.get_outputs().current_error.read()));
    packet.add_signal(34, static_cast<float>(current_controller.get_outputs().pi_output.read()));
    packet.add_signal(33, static_cast<float>(current_controller.get_outputs().is_active.read()));
    packet.add_signal(36, static_cast<float>(current_controller.get_outputs().is_saturated.read()));
    packet.add_signal(37, static_cast<float>(state_machine.get_outputs().preset_request.read()));
    packet.add_signal(35, static_cast<float>(mode_monitor.get_outputs().operating_mode.read()));

    // === State Machine Group ===
    packet.add_signal(38, static_cast<float>(static_cast<int>(state_machine.get_outputs().current_state_id.read())));
    packet.add_signal(39, state_machine.get_outputs().release_pulses.read() ? 1.0f : 0.0f);
    packet.add_signal(40, state_machine.get_outputs().auxiliaries_are_on.read() ? 1.0f : 0.0f);
    packet.add_signal(41, state_machine.get_outputs().breakers_are_on.read() ? 1.0f : 0.0f);
    packet.add_signal(42, state_machine.get_outputs().start_command.read() ? 1.0f : 0.0f);
    packet.add_signal(43, state_machine.get_outputs().stop_command.read() ? 1.0f : 0.0f);
    packet.add_signal(44, state_machine.get_outputs().is_current_extinguished.read() ? 1.0f : 0.0f);
    packet.add_signal(45, state_machine.get_outputs().shutdown_request.read() ? 1.0f : 0.0f);

    // === Operation Mode Monitor Group ===
    packet.add_signal(47, static_cast<float>(mode_monitor.get_outputs().operating_mode.read()));
    packet.add_signal(48, static_cast<float>(mode_monitor.get_outputs().has_mode_changed.read()));
    packet.add_signal(49, static_cast<float>(mode_monitor.get_outputs().is_current_zero.read()));
    packet.add_signal(50, static_cast<float>(mode_monitor.get_outputs().is_discontinuous_mode.read()));

    // === Pulse Generator Group ===
    const auto& firing_pulses = pulse_generator.get_outputs().firing_pulses.read();
    packet.add_signal(56, static_cast<float>(pulse_generator.get_outputs().release_pulses.read()));
    packet.add_signal(57, static_cast<float>(pulse_generator.get_outputs().actual_firing_angle.read().get_pu()));
    packet.add_signal(58, static_cast<float>(pulse_generator.get_inputs().release_pulses.read()));

    // Finalize packet with timestamp and sequence number
    packet.finalize(yeet::telemetry::packet_type::grid_side_telemetry, timestamp_ms, _sequence_number++);

    return packet;
}

}  // namespace yeet::application::sfc::telemetry
