/**
 * @file machine_side_telemetry_builder.cpp
 * @brief Implementation of machine-side telemetry packet builder
 */

#include "sfc_embedded/machine_side_network_telemetry_builder.hpp"

namespace yeet::application::sfc::telemetry {

yeet::telemetry::telemetry_packet machine_side_telemetry_builder::build_packet(std::uint32_t timestamp_ms) noexcept
{
    yeet::telemetry::telemetry_packet packet;

    // Extract control system components
    const auto& machine_meas = _control.get_machine_measurements();
    const auto& observer = _control.get_machine_side_observer();
    const auto& modulator = _control.get_machine_side_modulator();
    const auto& machine_controller = _control.get_machine_side_controller();
    const auto& torque_controller = _control.get_torque_controller();
    const auto& speed_controller = _control.get_speed_controller();
    const auto& excitation_controller = _control.get_excitation_controller();
    const auto& speed_generator = _control.get_speed_generator();
    const auto& pulse_generator = _control.get_machine_side_pulse_generator();

    // === Machine Measurements Group (0-9) ===
    // Voltage measurements
    packet.add_signal(0, static_cast<float>(machine_meas.get_outputs().machine_voltage_magnitude_filtered.read()));
    packet.add_signal(1, static_cast<float>(machine_meas.get_outputs().machine_voltage_abc.read().a()));
    packet.add_signal(2, static_cast<float>(machine_meas.get_outputs().machine_voltage_abc.read().b()));
    packet.add_signal(3, static_cast<float>(machine_meas.get_outputs().machine_voltage_abc.read().c()));

    // Speed and commutation status
    packet.add_signal(4, static_cast<float>(machine_meas.get_outputs().machine_side_speed.read()));
    packet.add_signal(5, static_cast<float>(machine_meas.get_outputs().machine_voltage_magnitude.read()));
    packet.add_signal(6, machine_meas.get_outputs().is_commuting.read() ? 1.0f : 0.0f);

    // Alpha-Beta frame measurements
    const auto& voltage_alphabeta = machine_meas.get_outputs().machine_voltage_alphabeta.read();
    packet.add_signal(7, static_cast<float>(voltage_alphabeta.alpha()));
    packet.add_signal(8, static_cast<float>(voltage_alphabeta.beta()));
    packet.add_signal(9, static_cast<float>(machine_meas.get_outputs().machine_voltage_magnitude_filtered.read()));

    // === Observer Group (10-14) ===
    packet.add_signal(10, static_cast<float>(observer.get_outputs().estimated_voltage_magnitude.read()));
    packet.add_signal(11, static_cast<float>(observer.get_outputs().estimated_frequency_pu.read()));
    packet.add_signal(12, static_cast<float>(observer.get_outputs().estimated_position.read().get_radians()));
    packet.add_signal(13, static_cast<float>(observer.get_outputs().estimated_frequency.read()));

    const auto& estimated_voltage_alphabeta = observer.get_outputs().filtered_voltage_alphabeta.read();
    packet.add_signal(14, static_cast<float>(estimated_voltage_alphabeta.alpha()));

    // === Modulator Group (15-19) ===
    packet.add_signal(15, static_cast<float>(modulator.get_outputs().firing_angle.read().get_radians()));
    packet.add_signal(16, modulator.get_inputs().enable.read() ? 1.0f : 0.0f);
    packet.add_signal(17, modulator.get_outputs().release_pulses.read() ? 1.0f : 0.0f);
    packet.add_signal(18, modulator.get_outputs().request_force_commutation.read() ? 1.0f : 0.0f);
    packet.add_signal(19, static_cast<float>(modulator.get_outputs().current_rotor_sector.read()));

    // === Machine-Side Controller Group (20-24) ===
    packet.add_signal(20, static_cast<float>(machine_controller.get_outputs().firing_angle.read().get_radians()));
    packet.add_signal(21, static_cast<float>(machine_controller.get_inputs().machine_speed.read()));
    packet.add_signal(22, static_cast<float>(machine_controller.get_inputs().dc_link_current.read()));
    packet.add_signal(23, 0.0f);  // Reserved
    packet.add_signal(24, 0.0f);  // Reserved

    // === Torque Controller Group (25-29) ===
    packet.add_signal(25, static_cast<float>(torque_controller.get_outputs().current_reference.read()));
    packet.add_signal(26, static_cast<float>(torque_controller.get_outputs().base_torque_reference.read()));
    packet.add_signal(27, static_cast<float>(torque_controller.get_outputs().total_torque_reference.read()));
    packet.add_signal(28, static_cast<float>(torque_controller.get_outputs().limited_torque_reference.read()));
    packet.add_signal(29, torque_controller.get_outputs().is_saturated.read() ? 1.0f : 0.0f);

    // === Speed Controller Group (30-34) ===
    packet.add_signal(30, static_cast<float>(speed_controller.get_outputs().torque_reference.read()));
    packet.add_signal(31, static_cast<float>(speed_controller.get_inputs().measured_speed_pu.read()));
    packet.add_signal(32, static_cast<float>(speed_controller.get_outputs().speed_error.read()));
    packet.add_signal(33, static_cast<float>(speed_controller.get_outputs().pi_output.read()));
    packet.add_signal(34, speed_controller.get_outputs().is_saturated.read() ? 1.0f : 0.0f);

    // === Speed Generator Group (35-39) ===
    packet.add_signal(35, static_cast<float>(speed_generator.get_outputs().speed_reference_pu.read()));
    packet.add_signal(36, static_cast<float>(speed_generator.get_outputs().torque_limit_max_pu.read()));
    packet.add_signal(37, static_cast<float>(speed_generator.get_outputs().torque_limit_min_pu.read()));
    packet.add_signal(38, speed_generator.get_outputs().is_profile_complete.read() ? 1.0f : 0.0f);
    packet.add_signal(39, speed_generator.get_outputs().is_breakaway_boost_active.read() ? 1.0f : 0.0f);

    // === Excitation Controller Group (40-44) ===
    packet.add_signal(
        40, static_cast<float>(excitation_controller.get_outputs().excitation_voltage_setpoint_pu.read()));
    packet.add_signal(41, excitation_controller.get_outputs().enable_excitation.read() ? 1.0f : 0.0f);
    packet.add_signal(42, excitation_controller.get_outputs().is_field_established.read() ? 1.0f : 0.0f);
    packet.add_signal(43, excitation_controller.get_outputs().is_running.read() ? 1.0f : 0.0f);
    packet.add_signal(44, 0.0f);  // Reserved

    // === Pulse Generator Group (45-49) ===
    const auto& firing_pulses = pulse_generator.get_outputs().firing_pulses.read();
    packet.add_signal(45, firing_pulses.p1 ? 1.0f : 0.0f);
    packet.add_signal(46, firing_pulses.p2 ? 1.0f : 0.0f);
    packet.add_signal(47, firing_pulses.p3 ? 1.0f : 0.0f);
    packet.add_signal(48, firing_pulses.p4 ? 1.0f : 0.0f);
    packet.add_signal(49, firing_pulses.p5 ? 1.0f : 0.0f);

    // Finalize packet with timestamp and sequence number
    packet.finalize(yeet::telemetry::packet_type::machine_side_telemetry, timestamp_ms, _sequence_number++);

    return packet;
}

}  // namespace yeet::application::sfc::telemetry
