/**
 * @file machine_side_telemetry_builder.cpp
 * @brief Implementation of machine-side telemetry packet builder
 */

#include "sfc_embedded/machine_side_network_telemetry_builder.hpp"
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
    const auto& startup_sequencer = _control.get_startup_sequencer();
    const auto& pulse_generator = _control.get_machine_side_modulator();

    // === Machine Measurements Group (0-6) ===
    // Voltage measurements
    packet.add_signal(0, static_cast<float>(measurements.get_outputs().machine_voltage_abc.read().a()));
    packet.add_signal(1, static_cast<float>(measurements.get_outputs().machine_voltage_abc.read().b()));
    packet.add_signal(2, static_cast<float>(measurements.get_outputs().machine_voltage_abc.read().c()));
    packet.add_signal(3, static_cast<float>(measurements.get_outputs().machine_voltage_magnitude_filtered.read()));
    packet.add_signal(4, static_cast<float>(observer.get_outputs().estimated_frequency_pu.read()));
    packet.add_signal(5, static_cast<float>(observer.get_outputs().estimated_voltage_magnitude.read()));

    // === Observer Group (6-9) ===
    packet.add_signal(6, static_cast<float>(observer.get_outputs().estimated_frequency_pu.read()));
    packet.add_signal(7, static_cast<float>(observer.get_outputs().estimated_reference_angle.read().get_pu()));
    packet.add_signal(8, static_cast<float>(observer.get_outputs().estimated_frequency.read()));
    packet.add_signal(9, static_cast<float>(observer.get_outputs().observer_error.read()));

    // === Modulator Group (10-16) ===
    packet.add_signal(10, static_cast<float>(modulator.get_inputs().release_pulses.read()));
    packet.add_signal(11, static_cast<float>(static_cast<int>(modulator.get_inputs().operating_mode.read())));

    // Pack 3 modulator status booleans (bits 0-2)
    packet.add_bool_pack(
        12, {modulator.get_outputs().release_pulses.read(), modulator.get_outputs().request_force_commutation.read(),
                modulator.get_inputs().force_commutation_acknowledged.read()});

    packet.add_signal(13, static_cast<float>(modulator.get_outputs().firing_angle.read().get_pu()));
    packet.add_signal(14, static_cast<float>(modulator.get_outputs().current_rotor_sector.read()));
    packet.add_signal(15, static_cast<float>(static_cast<int>(modulator.get_outputs().commutation_state.read())));

    // === Machine-Side Controller Group (16-18) ===
    packet.add_signal(16, static_cast<float>(machine_controller.get_outputs().firing_angle.read().get_pu()));
    packet.add_signal(17, static_cast<float>(machine_controller.get_inputs().dc_link_current.read()));
    packet.add_signal(18, static_cast<float>(machine_controller.get_inputs().machine_speed.read()));

    // === Torque Controller Group (19-23) ===
    packet.add_signal(19, static_cast<float>(torque_controller.get_inputs().torque_reference.read()));
    packet.add_signal(20, 0.0f);  // Reserved (was dynamic_max_torque_limit)
    packet.add_signal(21, 0.0f);  // Reserved (was dynamic_min_torque_limit)

    // Pack 5 torque controller status booleans (bits 0-4)
    packet.add_bool_pack(
        22, {torque_controller.get_inputs().enable_breakaway_boost.read(),
                torque_controller.get_inputs().is_saturated.read(), torque_controller.get_outputs().is_saturated.read(),
                torque_controller.get_outputs().is_active.read(),
                torque_controller.get_outputs().is_breakaway_boost_active.read()});

    packet.add_signal(23, static_cast<float>(torque_controller.get_outputs().current_reference.read()));

    // === Speed Controller Group (24-28) ===
    packet.add_signal(24, static_cast<float>(speed_controller.get_inputs().speed_reference.read()));
    packet.add_signal(25, static_cast<float>(speed_controller.get_inputs().measured_speed.read()));
    packet.add_signal(26, static_cast<float>(speed_controller.get_outputs().torque_reference.read()));
    packet.add_signal(27, static_cast<float>(speed_controller.get_outputs().pi_output.read()));

    // Pack speed controller saturated flag (bit 0)
    packet.add_bool_pack(28, {speed_controller.get_outputs().is_saturated.read()});

    // === Startup Sequencer Group (29-33) ===
    packet.add_signal(29, static_cast<float>(startup_sequencer.get_outputs().speed_reference.read()));
    packet.add_signal(30, static_cast<float>(startup_sequencer.get_outputs().virtual_rotor_frequency.read()));
    packet.add_signal(31, static_cast<float>(startup_sequencer.get_outputs().virtual_rotor_angle.read().get_pu()));
    packet.add_signal(32, static_cast<float>(startup_sequencer.get_outputs().virtual_torque_reference.read()));

    // Pack 5 startup sequencer release flags (bits 0-4)
    packet.add_bool_pack(33, {startup_sequencer.get_outputs().release_speed_controller.read(),
                                 startup_sequencer.get_outputs().release_excitation.read(),
                                 startup_sequencer.get_outputs().release_torque_controller.read(),
                                 startup_sequencer.get_outputs().release_machine_side_pulses.read(),
                                 startup_sequencer.get_outputs().is_startup_complete.read()});

    // === Excitation Controller Group (34-37) ===
    packet.add_signal(34, static_cast<float>(excitation_controller.get_outputs().field_current_reference.read()));
    packet.add_signal(35, static_cast<float>(excitation_controller.get_outputs().prefluxing_elapsed_time.read()));

    // === Excitation Voltage Control Diagnostics (36-38) ===
    packet.add_signal(36, static_cast<float>(excitation_controller.get_outputs().machine_voltage_reference_pu.read()));
    packet.add_signal(37, static_cast<float>(excitation_controller.get_outputs().machine_voltage_magnitude_pu.read()));
    packet.add_signal(38, static_cast<float>(excitation_controller.get_outputs().machine_voltage_error_pu.read()));

    // === Pulse Generator Group (39) ===
    packet.add_signal(39, static_cast<float>(pulse_generator.get_outputs().virtual_reference_angle.read().get_pu()));

    // === Machine-Side Protections Group (40-45) ===
    const auto& protections = _control.machine_protections;
    const auto& protection_params = protections.get_parameters();
    packet.add_signal(40,
        protection_status_to_signal(protection_params.over_flux_severity.read(),
            protections.get_outputs().over_flux_tripped.read(), protections.get_outputs().over_flux_alarmed.read()));
    packet.add_signal(41,
        protection_status_to_signal(protection_params.under_flux_severity.read(),
            protections.get_outputs().under_flux_tripped.read(), protections.get_outputs().under_flux_alarmed.read()));
    packet.add_signal(
        42, protection_status_to_signal(protection_params.stall_severity.read(),
                protections.get_outputs().stall_tripped.read(), protections.get_outputs().stall_alarmed.read()));
    packet.add_signal(43, protection_status_to_signal(protection_params.communication_loss_severity.read(),
                              protections.get_outputs().communication_loss_tripped.read(),
                              protections.get_outputs().communication_loss_alarmed.read()));
    packet.add_signal(44,
        protection_status_to_signal(protection_params.overspeed_severity.read(),
            protections.get_outputs().overspeed_tripped.read(), protections.get_outputs().overspeed_alarmed.read()));
    packet.add_signal(45, protection_status_to_signal(protection_params.machine_overvoltage_severity.read(),
                              protections.get_outputs().machine_overvoltage_tripped.read(),
                              protections.get_outputs().machine_overvoltage_alarmed.read()));

    // === Fiber Optic RX from Grid-Side Analog Group (46-48) ===
    const auto& fiber_rx = _control.external_ios.get_fiber_optics_inputs_from_grid();
    packet.add_signal(46, static_cast<float>(fiber_rx.dc_link_current.read()));
    packet.add_signal(47, static_cast<float>(fiber_rx.grid_voltage_magnitude.read()));
    packet.add_signal(48, static_cast<float>(static_cast<int>(fiber_rx.current_state_id.read())));

    // === Commutation Notch Duration Group (49-50) ===
    packet.add_signal(49, static_cast<float>(measurements.get_outputs().commutation_notch_duration_last.read()));
    packet.add_signal(50, static_cast<float>(measurements.get_outputs().commutation_notch_duration_filtered.read()));

    // === Grouped Boolean Packs (51-52) ===
    // Pack all machine-side booleans into 2 consecutive uint32 words for maximum compression
    // Bits 0-31 (Word 1): Modulator(3) + Torque(5) + Speed(1) + Startup(5) + PulseGen(2) + FiringPulses(6) +
    // Protections(9) Bits 32-36 (Word 2): Protections(2) + FiberOptic(4)
    const auto& firing_pulses = pulse_generator.get_outputs().firing_pulses.read();
    packet.add_bool_pack(51,
        {// Bits 0-2: Modulator status
            modulator.get_outputs().release_pulses.read(), modulator.get_outputs().request_force_commutation.read(),
            modulator.get_inputs().force_commutation_acknowledged.read(),
            // Bits 3-7: Torque controller
            torque_controller.get_inputs().enable_breakaway_boost.read(),
            torque_controller.get_inputs().is_saturated.read(), torque_controller.get_outputs().is_saturated.read(),
            torque_controller.get_outputs().is_active.read(),
            torque_controller.get_outputs().is_breakaway_boost_active.read(),
            // Bit 8: Speed controller
            speed_controller.get_outputs().is_saturated.read(),
            // Bits 9-13: Startup sequencer
            startup_sequencer.get_outputs().release_speed_controller.read(),
            startup_sequencer.get_outputs().release_excitation.read(),
            startup_sequencer.get_outputs().release_torque_controller.read(),
            startup_sequencer.get_outputs().release_machine_side_pulses.read(),
            startup_sequencer.get_outputs().is_startup_complete.read(),
            // Bits 14-15: Pulse generator
            pulse_generator.get_outputs().virtual_reference_active.read(),
            pulse_generator.get_outputs().individual_override_active.read(),
            // Bits 16-21: Firing pulses (p1-p6)
            firing_pulses.p1, firing_pulses.p2, firing_pulses.p3, firing_pulses.p4, firing_pulses.p5, firing_pulses.p6,
            // Bits 22-31: Protections (first 10)
            protections.get_outputs().has_active_trip.read(), protections.get_outputs().is_inhibited.read(),
            protections.get_outputs().has_active_alarm.read(), protections.get_outputs().over_flux_tripped.read(),
            protections.get_outputs().under_flux_tripped.read(), protections.get_outputs().stall_tripped.read(),
            protections.get_outputs().over_flux_warning.read(), protections.get_outputs().under_flux_warning.read(),
            protections.get_outputs().stall_warning.read(),
            protections.get_outputs().communication_loss_tripped.read()});

    packet.add_bool_pack(52,
        {// Bit 0: Protection (last 1)
            protections.get_outputs().communication_loss_warning.read(),
            // Bits 1-4: Fiber optic RX
            (fiber_rx.operating_mode.read() == sfc_operating_mode_t::forced_commutation),
            fiber_rx.force_commutation_acknowledged.read(), fiber_rx.release_pulses.read(),
            fiber_rx.grid_release_protections.read(),
            // Bits 5-14: Remaining Protections
            protections.get_outputs().overspeed_tripped.read(), protections.get_outputs().overspeed_alarmed.read(),
            protections.get_outputs().machine_overvoltage_tripped.read(),
            protections.get_outputs().machine_overvoltage_alarmed.read(),
            protections.get_outputs().loss_of_excitation_tripped.read(),
            protections.get_outputs().loss_of_excitation_alarmed.read(),
            protections.get_outputs().external_fault_tripped.read(),
            protections.get_outputs().external_fault_alarmed.read(),
            // Bit 14: Reserved/Machine Overvoltage Warning
            protections.get_outputs().machine_overvoltage_warning.read(),
            // Bits 15-18: Missing alarm bits
            protections.get_outputs().over_flux_alarmed.read(), protections.get_outputs().under_flux_alarmed.read(),
            protections.get_outputs().stall_alarmed.read(),
            protections.get_outputs().communication_loss_alarmed.read()});

    // === Line Voltages (Unmasked) Group (53-55) ===
    const auto& machine_voltage_line = measurements.get_outputs().machine_voltage_line.read();
    packet.add_signal(53, static_cast<float>(machine_voltage_line.ab()));
    packet.add_signal(54, static_cast<float>(machine_voltage_line.bc()));
    packet.add_signal(55, static_cast<float>(machine_voltage_line.ca()));

    // === Filtered Voltage Group (56-59) ===
    const auto& machine_voltage_abc_filtered = measurements.get_outputs().machine_voltage_abc_filtered.read();
    packet.add_signal(56, static_cast<float>(machine_voltage_abc_filtered.a()));
    packet.add_signal(57, static_cast<float>(machine_voltage_abc_filtered.b()));
    packet.add_signal(58, static_cast<float>(machine_voltage_abc_filtered.c()));
    const auto& machine_voltage_magnitude_filtered =
        measurements.get_outputs().machine_voltage_magnitude_filtered.read();
    packet.add_signal(59, static_cast<float>(machine_voltage_magnitude_filtered));

    // === Machine-Side Digital Inputs (60-61) ===
    const auto& machine_ios = _control.external_ios;
    const auto& machine_digital_inputs = machine_ios.get_digital_inputs();
    packet.add_signal(60, static_cast<float>(machine_digital_inputs.excitation_status.read()));
    packet.add_signal(61, static_cast<float>(machine_digital_inputs.external_fault.read()));

    // === Machine-Side Digital Outputs (62-63) ===
    // Use getter methods which apply polarity inversion and overrides
    packet.add_signal(62, static_cast<float>(machine_ios.get_excitation_on_command()));
    packet.add_signal(63, static_cast<float>(machine_ios.get_excitation_off_command()));

    // === Machine-Side Analog Outputs (64) ===
    // Use getter method which applies overrides and scaling
    packet.add_signal(64, static_cast<float>(machine_ios.get_excitation_current_reference()));

    // === Additional Machine-Side Protections (65-66) ===
    packet.add_signal(65, protection_status_to_signal(protection_params.loss_of_excitation_severity.read(),
                              protections.get_outputs().loss_of_excitation_tripped.read(),
                              protections.get_outputs().loss_of_excitation_alarmed.read()));
    packet.add_signal(66, static_cast<float>(protections.get_outputs().has_active_alarm.read()));

    // === Fiber Optic RX Boolean Signals (67-71) ===
    packet.add_signal(
        67, static_cast<float>((fiber_rx.operating_mode.read() == sfc_operating_mode_t::forced_commutation)));
    packet.add_signal(68, static_cast<float>(fiber_rx.force_commutation_acknowledged.read()));
    packet.add_signal(69, static_cast<float>(fiber_rx.release_pulses.read()));
    packet.add_signal(70, static_cast<float>(fiber_rx.grid_release_protections.read()));
    packet.add_signal(71, static_cast<float>(fiber_rx.communication_ok.read()));

    // === Intermediate Hardware Scale Signals (72-73) ===
    packet.add_signal(72, static_cast<float>(measurements.get_outputs().v_ab_counts_peak.read()));
    packet.add_signal(73, static_cast<float>(measurements.get_outputs().v_bc_counts_peak.read()));

    // Finalize packet with timestamp and sequence number
    packet.finalize(yeet::telemetry::packet_type::machine_side_telemetry, timestamp_ms, _sequence_number++);

    return packet;
}

}  // namespace yeet::application::sfc::telemetry
