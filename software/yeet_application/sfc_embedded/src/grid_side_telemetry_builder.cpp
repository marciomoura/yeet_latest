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
    const auto& pulse_generator = _control.get_grid_side_modulator();

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
    packet.add_signal(20, static_cast<float>(mode_monitor.get_outputs().operating_mode.read()));

    // === Current Controller Group (21-29) ===
    packet.add_signal(21, static_cast<float>(current_controller.get_inputs().current_reference.read()));
    packet.add_signal(22, static_cast<float>(current_controller.get_inputs().current_measured.read()));
    packet.add_signal(23, static_cast<float>(current_controller.get_inputs().freeze_integrator.read()));
    packet.add_signal(24, static_cast<float>(current_controller.get_inputs().preset_request.read()));
    packet.add_signal(25, static_cast<float>(current_controller.get_outputs().current_reference.read()));
    packet.add_signal(26, static_cast<float>(current_controller.get_outputs().current_error.read()));
    packet.add_signal(27, static_cast<float>(current_controller.get_outputs().pi_output.read()));

    // === State Machine Group (28-29) ===
    packet.add_signal(28, static_cast<float>(static_cast<int>(state_machine.get_outputs().current_state_id.read())));
    packet.add_signal(29, static_cast<float>(state_machine.get_outputs().preset_request.read()));

    // === Pulse Generator Group (30-31) ===
    packet.add_signal(30, static_cast<float>(pulse_generator.get_outputs().firing_angle.read().get_pu()));
    packet.add_signal(31, static_cast<float>(pulse_generator.get_outputs().virtual_reference_angle.read().get_pu()));

    // === Grid-Side Protections Severity/Status Group (32-36) ===
    const auto& protections = _control.grid_protections;
    const auto& protection_params = protections.get_parameters();
    packet.add_signal(32, protection_status_to_signal(protection_params.overcurrent_severity.read(),
                              protections.get_outputs().overcurrent_tripped.read(),
                              protections.get_outputs().overcurrent_alarmed.read()));
    packet.add_signal(33, protection_status_to_signal(protection_params.overvoltage_severity.read(),
                              protections.get_outputs().overvoltage_tripped.read(),
                              protections.get_outputs().overvoltage_alarmed.read()));
    packet.add_signal(34, protection_status_to_signal(protection_params.undervoltage_severity.read(),
                              protections.get_outputs().undervoltage_tripped.read(),
                              protections.get_outputs().undervoltage_alarmed.read()));
    packet.add_signal(35, protection_status_to_signal(protection_params.communication_loss_severity.read(),
                              protections.get_outputs().communication_loss_tripped.read(),
                              protections.get_outputs().communication_loss_alarmed.read()));
    packet.add_signal(36,
        protection_status_to_signal(protection_params.door_open_severity.read(),
            protections.get_outputs().door_open_tripped.read(), protections.get_outputs().door_open_alarmed.read()));
    // === Fiber Optic RX from Machine-Side Group (37-39) ===
    const auto& fiber_rx = _control.get_external_ios().get_fiber_optics_inputs_from_machine();
    packet.add_signal(37, static_cast<float>(fiber_rx.measured_speed.read()));
    packet.add_signal(38, static_cast<float>(fiber_rx.current_reference.read()));
    packet.add_signal(39, static_cast<float>(fiber_rx.machine_voltage_magnitude.read()));

    // === Commutation Notch Duration Group (40-41) ===
    packet.add_signal(40, static_cast<float>(grid_meas.get_outputs().commutation_notch_duration_last.read()));
    packet.add_signal(41, static_cast<float>(grid_meas.get_outputs().commutation_notch_duration_filtered.read()));

    const bool is_inhibited = _control.grid_protections.get_outputs().is_inhibited.read();
    const bool has_active_trip = _control.grid_protections.get_outputs().has_active_trip.read();
    const bool has_active_alarm = _control.grid_protections.get_outputs().has_active_alarm.read();

    // === Grouped Boolean Packs (42-43) ===
    // Pack all grid-side booleans into 2 consecutive uint32 words for maximum compression
    // Bits 0-31 (Word 1): Modulator(3) + CurrentCtrl(2) + StateMachine(7) + ModeMonitor(3) + PulseGen(4) +
    // FiringPulses(6) + Protections(7)
    // Bits 32-39 (Word 2): Protections(4) + FiberOptic(4)
    const auto& firing_pulses = pulse_generator.get_outputs().firing_pulses.read();
    packet.add_bool_pack(42,
        {// Bits 0-2: Modulator control
            modulator.get_outputs().release_pulses.read(),
            modulator.get_outputs().force_commutation_acknowledged.read(),
            modulator.get_outputs().freeze_integrator.read(),
            // Bits 3-4: Current controller status
            current_controller.get_outputs().is_active.read(), current_controller.get_outputs().is_saturated.read(),
            // Bits 5-10: State machine flags (6)
            state_machine.get_outputs().release_pulses.read(), state_machine.get_outputs().auxiliaries_are_on.read(),
            state_machine.get_outputs().mv_breaker_is_closed.read(), state_machine.get_outputs().start_command.read(),
            state_machine.get_outputs().stop_command.read(), state_machine.get_outputs().is_current_extinguished.read(),
            // Bits 11-13: Mode monitor
            mode_monitor.get_outputs().has_mode_changed.read(), mode_monitor.get_outputs().current_is_zero.read(),
            mode_monitor.get_outputs().is_discontinuous_mode.read(),
            // Bits 14-17: Pulse generator
            pulse_generator.get_outputs().release_pulses.read(), pulse_generator.get_inputs().release_pulses.read(),
            pulse_generator.get_outputs().virtual_reference_active.read(),
            pulse_generator.get_outputs().individual_override_active.read(),
            // Bits 18-23: Firing pulses (p1-p6)
            firing_pulses.p1, firing_pulses.p2, firing_pulses.p3, firing_pulses.p4, firing_pulses.p5, firing_pulses.p6,
            // Bits 24-30: Protections (first 7)
            has_active_trip, is_inhibited, has_active_alarm, protections.get_outputs().overcurrent_tripped.read(),
            protections.get_outputs().overvoltage_tripped.read(), protections.get_outputs().undervoltage_tripped.read(),
            protections.get_outputs().overcurrent_warning.read()});

    packet.add_bool_pack(43,
        {// Bits 0-3: Protections (remaining 4)
            protections.get_outputs().overvoltage_warning.read(), protections.get_outputs().undervoltage_warning.read(),
            protections.get_outputs().communication_loss_tripped.read(),
            protections.get_outputs().communication_loss_warning.read(),
            // Bits 4-6: Fiber optic RX
            fiber_rx.request_force_commutation.read(), fiber_rx.machine_protection_tripped.read(),
            fiber_rx.protection_inhibit.read(),
            // Bits 7-10: More trip bits
            protections.get_outputs().door_open_tripped.read(), protections.get_outputs().external_fault_tripped.read(),
            protections.get_outputs().emergency_off_tripped.read(),
            protections.get_outputs().machine_protection_tripped.read(),
            // Bits 11-18: Alarm bits
            protections.get_outputs().overcurrent_alarmed.read(), protections.get_outputs().overvoltage_alarmed.read(),
            protections.get_outputs().undervoltage_alarmed.read(),
            protections.get_outputs().communication_loss_alarmed.read(),
            protections.get_outputs().door_open_alarmed.read(), protections.get_outputs().external_fault_alarmed.read(),
            protections.get_outputs().emergency_off_alarmed.read(),
            protections.get_outputs().machine_protection_alarmed.read(),
            // Bit 19: Reset pulse
            protections.get_outputs().emergency_off_reset.read()});

    // === Line Voltages (Unmasked) Group (44-46) ===
    const auto& grid_voltage_line = grid_meas.get_outputs().grid_voltage_line.read();
    packet.add_signal(44, static_cast<float>(grid_voltage_line.ab()));
    packet.add_signal(45, static_cast<float>(grid_voltage_line.bc()));
    packet.add_signal(46, static_cast<float>(grid_voltage_line.ca()));

    // === Phase Currents Group (47-49) ===
    const auto& grid_current = grid_meas.get_outputs().grid_current.read();
    packet.add_signal(47, static_cast<float>(grid_current.a()));
    packet.add_signal(48, static_cast<float>(grid_current.b()));
    packet.add_signal(49, static_cast<float>(grid_current.c()));

    // === Filtered Voltage/Current Group (50-55) ===
    const auto& grid_voltage_abc_filtered = grid_meas.get_outputs().grid_voltage_abc_filtered.read();
    packet.add_signal(50, static_cast<float>(grid_voltage_abc_filtered.a()));
    packet.add_signal(51, static_cast<float>(grid_voltage_abc_filtered.b()));
    packet.add_signal(52, static_cast<float>(grid_voltage_abc_filtered.c()));
    const auto& grid_voltage_magnitude_filtered = grid_meas.get_outputs().grid_voltage_magnitude_filtered.read();
    packet.add_signal(53, static_cast<float>(grid_voltage_magnitude_filtered));
    // Current is not filtered in the runner, so we skip 54-55 for now
    packet.add_signal(54, 0.0f);  // Reserved for future filtered current
    packet.add_signal(55, 0.0f);  // Reserved for future filtered current

    // === Grid-Side Digital Inputs (56-64) ===
    const auto& grid_ios = _control.get_external_ios();
    const auto& grid_digital_inputs = grid_ios.get_digital_inputs();
    packet.add_signal(56, static_cast<float>(grid_digital_inputs.auxiliaries_are_on.read()));
    packet.add_signal(57, static_cast<float>(grid_digital_inputs.mv_breaker_is_closed.read()));
    packet.add_signal(58, static_cast<float>(grid_digital_inputs.start_command.read()));
    packet.add_signal(59, static_cast<float>(grid_digital_inputs.stop_command.read()));
    packet.add_signal(60, static_cast<float>(grid_digital_inputs.mv_breaker_is_tripping.read()));
    packet.add_signal(61, static_cast<float>(grid_digital_inputs.emergency_off.read()));
    packet.add_signal(62, static_cast<float>(grid_digital_inputs.door_is_open.read()));
    packet.add_signal(63, static_cast<float>(grid_digital_inputs.external_fault.read()));
    packet.add_signal(64, static_cast<float>(grid_digital_inputs.fans_are_running.read()));

    // === Grid-Side Digital Outputs (65-74) ===
    // Use getter methods which apply polarity inversion
    packet.add_signal(65, static_cast<float>(grid_ios.get_mv_breaker_close()));
    packet.add_signal(66, static_cast<float>(grid_ios.get_mv_breaker_open()));
    packet.add_signal(67, static_cast<float>(grid_ios.get_auxiliaries_on_command()));
    packet.add_signal(68, static_cast<float>(grid_ios.get_on_command_acu_fan_1()));
    packet.add_signal(69, static_cast<float>(grid_ios.get_on_command_acu_fan_2()));
    packet.add_signal(70, static_cast<float>(grid_ios.get_on_command_acu_fan_3()));
    packet.add_signal(71, static_cast<float>(grid_ios.get_on_command_acu_fan_4()));
    packet.add_signal(72, static_cast<float>(grid_ios.get_software_trip_1()));
    packet.add_signal(73, static_cast<float>(grid_ios.get_software_trip_2()));
    packet.add_signal(74, static_cast<float>(grid_ios.get_software_trip_3()));

    // === Additional Protection Status Signals (75-78) ===
    packet.add_signal(75, protection_status_to_signal(protection_params.external_fault_severity.read(),
                              protections.get_outputs().external_fault_tripped.read(),
                              protections.get_outputs().external_fault_alarmed.read()));
    packet.add_signal(76, protection_status_to_signal(protection_params.emergency_off_severity.read(),
                              protections.get_outputs().emergency_off_tripped.read(),
                              protections.get_outputs().emergency_off_alarmed.read()));
    packet.add_signal(77, protection_status_to_signal(protection_params.machine_protection_severity.read(),
                              protections.get_outputs().machine_protection_tripped.read(),
                              protections.get_outputs().machine_protection_alarmed.read()));
    packet.add_signal(78, static_cast<float>(protections.get_outputs().has_active_alarm.read()));

    // === Fiber Optic RX Boolean Signals (79-82) ===
    packet.add_signal(79, static_cast<float>(fiber_rx.request_force_commutation.read()));
    packet.add_signal(80, static_cast<float>(fiber_rx.machine_protection_tripped.read()));
    packet.add_signal(81, static_cast<float>(fiber_rx.protection_inhibit.read()));
    packet.add_signal(82, static_cast<float>(fiber_rx.communication_ok.read()));

    // === Intermediate Hardware Scale Signals (83-86) ===
    const auto& grid_outputs = grid_meas.get_outputs();
    packet.add_signal(83, static_cast<float>(grid_outputs.v_ab_counts_peak.read()));
    packet.add_signal(84, static_cast<float>(grid_outputs.v_bc_counts_peak.read()));
    packet.add_signal(85, static_cast<float>(grid_outputs.i_a_counts_peak.read()));
    packet.add_signal(86, static_cast<float>(grid_outputs.i_c_counts_peak.read()));

    // === First Trip Source (87) ===
    packet.add_signal(
        87, static_cast<float>(static_cast<uint32_t>(protections.get_outputs().first_trip_source.read())));

    // Finalize packet with timestamp and sequence number
    packet.finalize(yeet::telemetry::packet_type::grid_side_telemetry, timestamp_ms, _sequence_number++);

    return packet;
}

}  // namespace yeet::application::sfc::telemetry
