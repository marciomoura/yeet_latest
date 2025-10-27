/**
 * @file network_telemetry_builder.cpp
 * @brief Implementation of telemetry packet builders
 */

#include "sfc_embedded/network_telemetry_builder.hpp"

namespace yeet::application::sfc::telemetry {

grid_side_telemetry_packet grid_side_telemetry_builder::build_packet(std::uint32_t timestamp_ms) noexcept
{
    grid_side_telemetry_packet packet;

    // Extract control system components
    const auto& grid_meas = _control.get_grid_measurements();
    const auto& dc_meas = _control.get_dc_link_measurements();
    const auto& observer = _control.get_grid_side_observer();
    const auto& modulator = _control.get_grid_side_modulator();
    const auto& controller = _control.get_current_controller();
    const auto& mode_monitor = _control.get_operation_mode_monitor();
    const auto& state_machine = _control.get_state_machine();
    const auto& pulse_generator = _control.get_grid_side_pulse_generator();

    // === Grid Measurements Group ===
    // ABC frame measurements
    packet.push_signal(static_cast<float>(grid_meas.get_outputs().grid_voltage_magnitude_filtered.read()));
    packet.push_signal(static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().a()));
    packet.push_signal(static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().b()));
    packet.push_signal(static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().c()));

    const auto& grid_current = grid_meas.get_outputs().grid_current_abc.read();
    packet.push_signal(static_cast<float>(grid_current.to_alphabeta().magnitude()));  // Grid current magnitude
    packet.push_signal(static_cast<float>(grid_current.a()));
    packet.push_signal(static_cast<float>(grid_current.b()));
    packet.push_signal(static_cast<float>(grid_current.c()));

    packet.push_signal(static_cast<float>(dc_meas.get_outputs().dc_current_pu.read()));
    packet.push_signal(grid_meas.get_outputs().is_commuting.read() ? 1.0f : 0.0f);

    // Alpha-Beta frame and filtered measurements
    const auto& voltage_alphabeta = grid_meas.get_outputs().grid_voltage_alphabeta.read();
    packet.push_signal(static_cast<float>(voltage_alphabeta.alpha()));
    packet.push_signal(static_cast<float>(voltage_alphabeta.beta()));
    packet.push_signal(static_cast<float>(grid_meas.get_outputs().grid_voltage_magnitude_filtered.read()));
    packet.push_signal(static_cast<float>(grid_meas.get_outputs().grid_side_frequency.read()));

    // === Observer Group ===
    packet.push_signal(static_cast<float>(observer.get_outputs().estimated_voltage_magnitude.read()));
    packet.push_signal(static_cast<float>(observer.get_outputs().estimated_frequency_pu.read()));
    packet.push_signal(static_cast<float>(observer.get_outputs().estimated_position.read().get_radians()));
    packet.push_signal(static_cast<float>(observer.get_outputs().estimated_frequency.read()));

    const auto& estimated_voltage_alphabeta = observer.get_outputs().filtered_voltage_alphabeta.read();
    packet.push_signal(static_cast<float>(estimated_voltage_alphabeta.alpha()));
    packet.push_signal(static_cast<float>(estimated_voltage_alphabeta.beta()));

    // === Modulator Group ===
    packet.push_signal(static_cast<float>(modulator.get_outputs().firing_angle.read().get_radians()));
    packet.push_signal(modulator.get_inputs().enable.read() ? 1.0f : 0.0f);
    packet.push_signal(modulator.get_outputs().force_commutation_acknowledged.read() ? 1.0f : 0.0f);
    packet.push_signal(modulator.get_outputs().is_requesting_main_pi_reset.read() ? 1.0f : 0.0f);

    // === Controller Group ===
    packet.push_signal(static_cast<float>(controller.get_inputs().dc_link_current_reference.read()));
    packet.push_signal(static_cast<float>(controller.get_inputs().dc_link_current_measured.read()));
    packet.push_signal(static_cast<float>(controller.get_outputs().current_error.read()));
    packet.push_signal(static_cast<float>(controller.get_outputs().pi_output.read().get_radians()));

    const auto operating_mode = mode_monitor.get_outputs().operating_mode.read();
    packet.push_signal((operating_mode == sfc::sfc_operating_mode_t::forced_commutation) ? 0.0f : 1.0f);
    packet.push_signal(controller.get_outputs().current_controller_is_saturated.read() ? 1.0f : 0.0f);
    packet.push_signal(state_machine.get_outputs().freeze_integrator.read() ? 1.0f : 0.0f);

    // === State Machine Group ===
    packet.push_signal(static_cast<float>(state_machine.get_outputs().current_state_id.read()));
    packet.push_signal(state_machine.get_outputs().pulse_releases.read() ? 1.0f : 0.0f);
    packet.push_signal(state_machine.get_outputs().auxiliaries_are_on.read() ? 1.0f : 0.0f);
    packet.push_signal(state_machine.get_outputs().breakers_are_on.read() ? 1.0f : 0.0f);
    packet.push_signal(state_machine.get_outputs().start_command.read() ? 1.0f : 0.0f);
    packet.push_signal(state_machine.get_outputs().stop_command.read() ? 1.0f : 0.0f);
    packet.push_signal(state_machine.get_outputs().is_current_extinguished.read() ? 1.0f : 0.0f);
    packet.push_signal(state_machine.get_outputs().shutdown_request.read() ? 1.0f : 0.0f);

    // === Operation Mode Monitor Group ===
    packet.push_signal((operating_mode == sfc::sfc_operating_mode_t::forced_commutation) ? 0.0f : 1.0f);
    packet.push_signal(mode_monitor.get_outputs().has_mode_changed.read() ? 1.0f : 0.0f);
    packet.push_signal(mode_monitor.get_outputs().is_current_zero.read() ? 1.0f : 0.0f);
    packet.push_signal(mode_monitor.get_outputs().is_discontinuous_mode.read() ? 1.0f : 0.0f);

    // === Pulse Generator Group ===
    const auto& firing_pulses = pulse_generator.get_outputs().firing_pulses.read();
    packet.push_signal(firing_pulses.p1 ? 1.0f : 0.0f);
    packet.push_signal(firing_pulses.p2 ? 1.0f : 0.0f);
    packet.push_signal(firing_pulses.p3 ? 1.0f : 0.0f);
    packet.push_signal(firing_pulses.p4 ? 1.0f : 0.0f);
    packet.push_signal(firing_pulses.p5 ? 1.0f : 0.0f);
    packet.push_signal(firing_pulses.p6 ? 1.0f : 0.0f);
    packet.push_signal(pulse_generator.get_outputs().is_active.read() ? 1.0f : 0.0f);

    // Finalize packet with timestamp and sequence number
    packet.finalize(timestamp_ms, _sequence_number++);

    return packet;
}

}  // namespace yeet::application::sfc::telemetry
