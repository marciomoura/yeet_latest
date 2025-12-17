/**
 * @file grid_side_telemetry_builder.cpp
 * @brief Implementation of grid-side telemetry packet builder
 */

#include "sfc_embedded/grid_side_network_telemetry_builder.hpp"
#include "sfc_integration/protection_severity.hpp"

namespace yeet::application::sfc::telemetry {

namespace {

float protection_status_to_signal(
    yeet::application::sfc::integration::protection_severity severity, bool tripped, bool alarmed) noexcept
{
    const bool is_trip = (severity == yeet::application::sfc::integration::protection_severity::trip);
    const bool is_active = is_trip ? tripped : alarmed;

    // 0: TRIP OFF, 1: TRIP ON, 2: ALARM OFF, 3: ALARM ON
    if (is_trip) {
        return is_active ? 1.0f : 0.0f;
    }
    return is_active ? 3.0f : 2.0f;
}

}  // namespace

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

    // === Grid Measurements Group (0-8) ===
    // ABC frame measurements
    packet.add_signal(0, static_cast<float>(grid_meas.get_outputs().grid_voltage_magnitude_filtered.read()));
    packet.add_signal(1, static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().a()));
    packet.add_signal(2, static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().b()));
    packet.add_signal(3, static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().c()));

    const auto& grid_current_abc = grid_meas.get_outputs().grid_current_abc.read();
    packet.add_signal(4, static_cast<float>(grid_current_abc.to_alphabeta().magnitude()));  // Grid current magnitude
    packet.add_signal(5, static_cast<float>(grid_current_abc.a()));
    packet.add_signal(6, static_cast<float>(grid_current_abc.b()));
    packet.add_signal(7, static_cast<float>(grid_current_abc.c()));
    packet.add_signal(8, static_cast<float>(dc_meas.get_outputs().dc_current_filtered_pu.read()));

    // === Observer Group (9-12) ===
    packet.add_signal(9, static_cast<float>(observer.get_outputs().estimated_voltage_magnitude.read()));
    packet.add_signal(10, static_cast<float>(observer.get_outputs().estimated_frequency_pu.read()));
    packet.add_signal(11, static_cast<float>(observer.get_outputs().estimated_reference_angle.read().get_pu()));
    packet.add_signal(12, static_cast<float>(observer.get_outputs().observer_error.read()));

    // === Modulator Group (13-20) ===
    packet.add_signal(13, static_cast<float>(modulator.get_inputs().release_pulses.read()));
    packet.add_signal(14, static_cast<float>(modulator.get_inputs().firing_angle.read().get_pu()));
    packet.add_signal(15, static_cast<float>(modulator.get_outputs().firing_angle.read().get_pu()));
    packet.add_signal(16, static_cast<float>(modulator.get_inputs().operating_mode.read()));
    packet.add_signal(17, static_cast<float>(modulator.get_inputs().request_extinguish_dc_current.read()));
    packet.add_signal(18, static_cast<float>(modulator.get_inputs().request_force_commutation.read()));
    packet.add_signal(19, static_cast<float>(modulator.get_outputs().modulator_state.read()));
    packet.add_signal(20, modulator.get_outputs().release_pulses.read() ? 1.0f : 0.0f);  // Modulator outputs

    // === Modulator Control Group (20-22) ===
    packet.add_signal(21, modulator.get_outputs().force_commutation_acknowledged.read() ? 1.0f : 0.0f);
    packet.add_signal(22, modulator.get_outputs().freeze_integrator.read() ? 1.0f : 0.0f);
    packet.add_signal(23, static_cast<float>(mode_monitor.get_outputs().operating_mode.read()));

    // === Current Controller Group (24-32) ===
    packet.add_signal(24, static_cast<float>(current_controller.get_inputs().current_reference.read()));
    packet.add_signal(25, static_cast<float>(current_controller.get_inputs().current_measured.read()));
    packet.add_signal(26, static_cast<float>(current_controller.get_inputs().freeze_integrator.read()));
    packet.add_signal(27, static_cast<float>(current_controller.get_inputs().preset_request.read()));
    packet.add_signal(28, static_cast<float>(current_controller.get_outputs().current_reference.read()));
    packet.add_signal(29, static_cast<float>(current_controller.get_outputs().current_error.read()));
    packet.add_signal(30, static_cast<float>(current_controller.get_outputs().pi_output.read()));
    packet.add_signal(31, static_cast<float>(current_controller.get_outputs().is_active.read()));
    packet.add_signal(32, static_cast<float>(current_controller.get_outputs().is_saturated.read()));

    // === State Machine Group (33-40) ===
    packet.add_signal(33, static_cast<float>(static_cast<int>(state_machine.get_outputs().current_state_id.read())));
    packet.add_signal(34, state_machine.get_outputs().release_pulses.read() ? 1.0f : 0.0f);
    packet.add_signal(35, state_machine.get_outputs().auxiliaries_are_on.read() ? 1.0f : 0.0f);
    packet.add_signal(36, state_machine.get_outputs().mcb_is_closed.read() ? 1.0f : 0.0f);
    packet.add_signal(37, state_machine.get_outputs().start_command.read() ? 1.0f : 0.0f);
    packet.add_signal(38, state_machine.get_outputs().stop_command.read() ? 1.0f : 0.0f);
    packet.add_signal(39, state_machine.get_outputs().is_current_extinguished.read() ? 1.0f : 0.0f);
    packet.add_signal(40, state_machine.get_outputs().shutdown_request.read() ? 1.0f : 0.0f);

    // === Operation Mode Monitor Group (41-44) ===
    packet.add_signal(41, static_cast<float>(state_machine.get_outputs().preset_request.read()));
    packet.add_signal(42, static_cast<float>(mode_monitor.get_outputs().has_mode_changed.read()));
    packet.add_signal(43, static_cast<float>(mode_monitor.get_outputs().current_is_zero.read()));
    packet.add_signal(44, static_cast<float>(mode_monitor.get_outputs().is_discontinuous_mode.read()));

    // === Pulse Generator Group (45-56) ===
    packet.add_signal(45, static_cast<float>(pulse_generator.get_outputs().release_pulses.read()));
    packet.add_signal(46, static_cast<float>(pulse_generator.get_outputs().actual_firing_angle.read().get_pu()));
    packet.add_signal(47, static_cast<float>(pulse_generator.get_inputs().release_pulses.read()));
    packet.add_signal(48, pulse_generator.get_outputs().virtual_reference_active.read() ? 1.0f : 0.0f);
    packet.add_signal(49, static_cast<float>(pulse_generator.get_outputs().virtual_reference_angle.read().get_pu()));
    packet.add_signal(50, pulse_generator.get_outputs().individual_override_active.read() ? 1.0f : 0.0f);
    const auto& firing_pulses = pulse_generator.get_outputs().firing_pulses.read();
    packet.add_signal(51, firing_pulses.p1 ? 1.0f : 0.0f);
    packet.add_signal(52, firing_pulses.p2 ? 1.0f : 0.0f);
    packet.add_signal(53, firing_pulses.p3 ? 1.0f : 0.0f);
    packet.add_signal(54, firing_pulses.p4 ? 1.0f : 0.0f);
    packet.add_signal(55, firing_pulses.p5 ? 1.0f : 0.0f);
    packet.add_signal(56, firing_pulses.p6 ? 1.0f : 0.0f);

    // === Grid-Side Protections Group (57-67) ===
    const auto& protections = _control.grid_protections;
    packet.add_signal(57, protections.get_outputs().is_tripped.read() ? 1.0f : 0.0f);
    packet.add_signal(58, protections.get_outputs().is_inhibited.read() ? 1.0f : 0.0f);
    packet.add_signal(59, protections.get_outputs().any_fault_active.read() ? 1.0f : 0.0f);
    packet.add_signal(60, protections.get_outputs().overcurrent_tripped.read() ? 1.0f : 0.0f);
    packet.add_signal(61, protections.get_outputs().overvoltage_tripped.read() ? 1.0f : 0.0f);
    packet.add_signal(62, protections.get_outputs().undervoltage_tripped.read() ? 1.0f : 0.0f);
    packet.add_signal(63, protections.get_outputs().overcurrent_warning.read() ? 1.0f : 0.0f);
    packet.add_signal(64, protections.get_outputs().overvoltage_warning.read() ? 1.0f : 0.0f);
    packet.add_signal(65, protections.get_outputs().undervoltage_warning.read() ? 1.0f : 0.0f);
    packet.add_signal(66, protections.get_outputs().communication_loss_tripped.read() ? 1.0f : 0.0f);
    packet.add_signal(67, protections.get_outputs().communication_loss_warning.read() ? 1.0f : 0.0f);

    // === Fiber Optic RX from Machine-Side Group (68-75) ===
    const auto& fiber_rx = _control.get_external_ios().get_fiber_optics_inputs_from_machine();
    packet.add_signal(68, static_cast<float>(fiber_rx.measured_speed.read()));
    packet.add_signal(69, static_cast<float>(fiber_rx.current_reference.read()));
    packet.add_signal(70, static_cast<float>(fiber_rx.machine_voltage_magnitude.read()));
    packet.add_signal(71, fiber_rx.request_force_commutation.read() ? 1.0f : 0.0f);
    packet.add_signal(72, fiber_rx.shutdown_request.read() ? 1.0f : 0.0f);
    packet.add_signal(73, fiber_rx.machine_protection_tripped.read() ? 1.0f : 0.0f);
    packet.add_signal(74, fiber_rx.protection_inhibit.read() ? 1.0f : 0.0f);
    packet.add_signal(75, fiber_rx.machine_trip_acknowledged.read() ? 1.0f : 0.0f);

    // === Grid-Side Protections Severity/Status Group (76-80) ===
    const auto& protection_params = protections.get_parameters();
    packet.add_signal(76, protection_status_to_signal(protection_params.overcurrent_severity.read(),
                              protections.get_outputs().overcurrent_tripped.read(),
                              protections.get_outputs().overcurrent_alarmed.read()));
    packet.add_signal(77, protection_status_to_signal(protection_params.overvoltage_severity.read(),
                              protections.get_outputs().overvoltage_tripped.read(),
                              protections.get_outputs().overvoltage_alarmed.read()));
    packet.add_signal(78, protection_status_to_signal(protection_params.undervoltage_severity.read(),
                              protections.get_outputs().undervoltage_tripped.read(),
                              protections.get_outputs().undervoltage_alarmed.read()));
    packet.add_signal(79, protection_status_to_signal(protection_params.communication_loss_severity.read(),
                              protections.get_outputs().communication_loss_tripped.read(),
                              protections.get_outputs().communication_loss_alarmed.read()));
    packet.add_signal(80,
        protection_status_to_signal(protection_params.door_open_severity.read(),
            protections.get_outputs().door_open_tripped.read(), protections.get_outputs().door_open_alarmed.read()));

    // === Commutation Notch Duration Group (81-82) ===
    packet.add_signal(81, static_cast<float>(grid_meas.get_outputs().commutation_notch_duration_last.read()));
    packet.add_signal(82, static_cast<float>(grid_meas.get_outputs().commutation_notch_duration_filtered.read()));

    // === Line Voltages (Unmasked) Group (83-85) ===
    const auto& grid_voltage_line = grid_meas.get_outputs().grid_voltage_line.read();
    packet.add_signal(83, static_cast<float>(grid_voltage_line.ab()));
    packet.add_signal(84, static_cast<float>(grid_voltage_line.bc()));
    packet.add_signal(85, static_cast<float>(grid_voltage_line.ca()));

    // === Phase Currents Group (86-88) ===
    const auto& grid_current = grid_meas.get_outputs().grid_current.read();
    packet.add_signal(86, static_cast<float>(grid_current.a()));
    packet.add_signal(87, static_cast<float>(grid_current.b()));
    packet.add_signal(88, static_cast<float>(grid_current.c()));

    // === Filtered Voltage/Current Group (89-94) ===
    const auto& grid_voltage_abc_filtered = grid_meas.get_outputs().grid_voltage_abc_filtered.read();
    packet.add_signal(89, static_cast<float>(grid_voltage_abc_filtered.a()));
    packet.add_signal(90, static_cast<float>(grid_voltage_abc_filtered.b()));
    packet.add_signal(91, static_cast<float>(grid_voltage_abc_filtered.c()));
    const auto& grid_voltage_magnitude_filtered = grid_meas.get_outputs().grid_voltage_magnitude_filtered.read();
    packet.add_signal(92, static_cast<float>(grid_voltage_magnitude_filtered));
    // Current is not filtered in the runner, so we skip 93-94 for now
    packet.add_signal(93, 0.0f);  // Reserved for future filtered current
    packet.add_signal(94, 0.0f);  // Reserved for future filtered current

    // Finalize packet with timestamp and sequence number
    packet.finalize(yeet::telemetry::packet_type::grid_side_telemetry, timestamp_ms, _sequence_number++);

    return packet;
}

}  // namespace yeet::application::sfc::telemetry
