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
    const auto& measurements = _control.get_machine_measurements();
    const auto& observer = _control.get_machine_side_observer();
    const auto& modulator = _control.get_machine_side_modulator();
    const auto& machine_controller = _control.get_machine_side_controller();
    const auto& torque_controller = _control.get_torque_controller();
    const auto& speed_controller = _control.get_speed_controller();
    const auto& excitation_controller = _control.get_excitation_controller();
    const auto& speed_generator = _control.get_speed_generator();
    const auto& pulse_generator = _control.get_machine_side_pulse_generator();

    // === Machine Measurements Group (0-6) ===
    // Voltage measurements
    packet.add_signal(0, static_cast<float>(measurements.get_outputs().machine_voltage_abc.read().a()));
    packet.add_signal(1, static_cast<float>(measurements.get_outputs().machine_voltage_abc.read().b()));
    packet.add_signal(2, static_cast<float>(measurements.get_outputs().machine_voltage_abc.read().c()));
    packet.add_signal(3, static_cast<float>(measurements.get_outputs().machine_voltage_magnitude_filtered.read()));
    packet.add_signal(4, static_cast<float>(observer.get_outputs().estimated_speed.read()));
    packet.add_signal(5, static_cast<float>(observer.get_outputs().estimated_voltage_magnitude.read()));

    // === Observer Group (6-8) ===
    packet.add_signal(6, static_cast<float>(observer.get_outputs().estimated_frequency_pu.read()));
    packet.add_signal(7, static_cast<float>(observer.get_outputs().estimated_reference_angle.read().get_pu()));
    packet.add_signal(8, static_cast<float>(observer.get_outputs().estimated_frequency.read()));

    // === Modulator Group (9-16) ===
    packet.add_signal(9, static_cast<float>(modulator.get_inputs().release_pulses.read()));
    packet.add_signal(10, static_cast<float>(static_cast<int>(
                              modulator.get_inputs().operating_mode.read())));           // Operating mode indicator
    packet.add_signal(11, modulator.get_outputs().release_pulses.read() ? 1.0f : 0.0f);  // Are pulses released
    packet.add_signal(12, modulator.get_outputs().request_force_commutation.read()
                              ? 1.0f
                              : 0.0f);  // Force commutation request (to grid side)
    packet.add_signal(13,
        modulator.get_inputs().force_commutation_acknowledged.read()
            ? 1.0f
            : 0.0f);  // Force commutation acknowledged (from grid side)
    packet.add_signal(14, static_cast<float>(modulator.get_outputs().firing_angle.read().get_pu()));
    packet.add_signal(15, static_cast<float>(modulator.get_outputs().current_rotor_sector.read()));
    packet.add_signal(16, static_cast<float>(static_cast<int>(modulator.get_outputs().commutation_state.read())));

    // === Machine-Side Controller Group (17-19) ===
    packet.add_signal(17, static_cast<float>(machine_controller.get_outputs().firing_angle.read().get_pu()));
    packet.add_signal(18, static_cast<float>(machine_controller.get_inputs().dc_link_current.read()));
    packet.add_signal(19, static_cast<float>(machine_controller.get_inputs().machine_speed.read()));

    // === Torque Controller Group (20-28) ===
    packet.add_signal(20, static_cast<float>(torque_controller.get_inputs().torque_reference.read()));
    packet.add_signal(21, static_cast<float>(torque_controller.get_inputs().dynamic_max_torque_limit.read()));
    packet.add_signal(22, static_cast<float>(torque_controller.get_inputs().dynamic_min_torque_limit.read()));
    packet.add_signal(23, torque_controller.get_inputs().enable_breakaway_boost.read() ? 1.0f : 0.0f);
    packet.add_signal(24, torque_controller.get_inputs().is_saturated.read() ? 1.0f : 0.0f);
    packet.add_signal(25, static_cast<float>(torque_controller.get_outputs().current_reference.read()));
    packet.add_signal(26, torque_controller.get_outputs().is_saturated.read() ? 1.0f : 0.0f);
    packet.add_signal(27, torque_controller.get_outputs().is_active.read() ? 1.0f : 0.0f);
    packet.add_signal(28, torque_controller.get_outputs().is_breakaway_boost_active.read() ? 1.0f : 0.0f);

    // === Speed Controller Group (29-33) ===
    packet.add_signal(29, static_cast<float>(speed_controller.get_inputs().speed_reference.read()));
    packet.add_signal(30, static_cast<float>(speed_controller.get_inputs().measured_speed.read()));
    packet.add_signal(31, static_cast<float>(speed_controller.get_outputs().torque_reference.read()));
    packet.add_signal(32, static_cast<float>(speed_controller.get_outputs().pi_output.read()));
    packet.add_signal(33, speed_controller.get_outputs().is_saturated.read() ? 1.0f : 0.0f);

    // === Speed Generator Group (34-38) ===
    packet.add_signal(34, static_cast<float>(speed_generator.get_outputs().speed_reference.read()));
    packet.add_signal(35, static_cast<float>(speed_generator.get_outputs().torque_limit_max_pu.read()));
    packet.add_signal(36, static_cast<float>(speed_generator.get_outputs().torque_limit_min_pu.read()));
    packet.add_signal(37, speed_generator.get_outputs().is_profile_complete.read() ? 1.0f : 0.0f);
    packet.add_signal(38, speed_generator.get_outputs().is_breakaway_boost_active.read() ? 1.0f : 0.0f);

    // === Excitation Controller Group (39-42) ===
    packet.add_signal(39, static_cast<float>(excitation_controller.get_outputs().excitation_voltage_setpoint.read()));
    packet.add_signal(40, excitation_controller.get_outputs().enable_excitation.read() ? 1.0f : 0.0f);
    packet.add_signal(41, excitation_controller.get_outputs().is_field_established.read() ? 1.0f : 0.0f);
    packet.add_signal(42, excitation_controller.get_outputs().is_running.read() ? 1.0f : 0.0f);

    // Finalize packet with timestamp and sequence number
    packet.finalize(yeet::telemetry::packet_type::machine_side_telemetry, timestamp_ms, _sequence_number++);

    return packet;
}

}  // namespace yeet::application::sfc::telemetry
