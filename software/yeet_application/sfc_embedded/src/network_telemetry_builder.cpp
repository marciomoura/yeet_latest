/**
 * @file network_telemetry_builder.cpp
 * @brief Implementation of telemetry packet builders
 */

#include "sfc_embedded/network_telemetry_builder.hpp"

namespace yeet::application::sfc::telemetry {

void grid_side_telemetry_builder::populate_measurements(grid_measurements_group& group) const noexcept
{
    const auto& grid_meas = _control.get_grid_measurements();
    const auto& dc_meas = _control.get_dc_link_measurements();

    // Grid voltage measurements
    group.is_commuting = grid_meas.get_outputs().is_commuting.read();
    group.grid_voltage_magnitude_pu =
        static_cast<float>(grid_meas.get_outputs().grid_voltage_magnitude_filtered.read());
    group.grid_voltage_a_pu = static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().a());
    group.grid_voltage_b_pu = static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().b());
    group.grid_voltage_c_pu = static_cast<float>(grid_meas.get_outputs().grid_voltage_abc.read().c());

    // DC link measurements
    group.dc_link_current_pu = static_cast<float>(dc_meas.get_outputs().dc_current_pu.read());
}

void grid_side_telemetry_builder::populate_observer(grid_observer_group& group) const noexcept
{
    const auto& observer = _control.get_grid_side_observer();

    group.estimated_voltage_magnitude_pu =
        static_cast<float>(observer.get_outputs().estimated_voltage_magnitude.read());
    group.estimated_frequency_pu = static_cast<float>(observer.get_outputs().estimated_frequency_pu.read());
    group.estimated_angle_rad = static_cast<float>(observer.get_outputs().estimated_position.read().get_radians());
}

void grid_side_telemetry_builder::populate_modulator(grid_modulator_group& group) const noexcept
{
    const auto& modulator = _control.get_grid_side_modulator();
    // Firing angle (same value for reference and actual - no separate saturated output available)
    group.is_enabled = modulator.get_inputs().enable.read();
    group.firing_angle_input_pu = static_cast<float>(modulator.get_inputs().firing_angle.read().get_pu());
    group.firing_angle_output_pu = static_cast<float>(modulator.get_outputs().firing_angle.read().get_pu());
    group.force_commutation_ack = modulator.get_outputs().force_commutation_acknowledged.read();
}

void grid_side_telemetry_builder::populate_controller(grid_controller_group& group) const noexcept
{
    const auto& controller = _control.get_current_controller();
    const auto& mode_monitor = _control.get_operation_mode_monitor();

    group.dc_link_current_measured = controller.get_inputs().dc_link_current_measured.read();
    group.dc_link_current_reference = controller.get_inputs().dc_link_current_reference.read();
    // Controller outputs
    group.current_error_pu = static_cast<float>(controller.get_outputs().current_error.read());
    group.controller_output_pu = static_cast<float>(controller.get_outputs().pi_output.read().get_radians());

    // Operating mode
    const auto operating_mode = mode_monitor.get_outputs().operating_mode.read();
    group.operating_mode = (operating_mode == sfc::sfc_operating_mode_t::forced_commutation) ? 0 : 1;

    group.controller_saturated = controller.get_outputs().current_controller_is_saturated.read() ? 1 : 0;
    group.freeze_integrator = _control.get_state_machine().get_outputs().freeze_integrator.read() ? 1 : 0;
    group.reserved = 0;
}

void grid_side_telemetry_builder::populate_state_machine(state_machine_group& group) const noexcept
{
    const auto& state_machine = _control.get_state_machine();

    // State machine outputs
    group.current_state = static_cast<std::uint8_t>(state_machine.get_outputs().current_state_id.read());
    group.pulse_releases = state_machine.get_outputs().pulse_releases.read();
    group.auxiliaries_are_on = state_machine.get_inputs().auxiliaries_are_on.read();
    group.breakers_are_on = state_machine.get_inputs().breakers_are_on.read();
    group.start_command = state_machine.get_inputs().start_command.read();
    group.stop_command = state_machine.get_inputs().stop_command.read();
    group.is_current_extinguished = state_machine.get_inputs().is_current_extinguished.read();
    group.is_shutdown_requested = state_machine.get_outputs().is_shutdown_requested.read();
}

grid_side_telemetry_packet grid_side_telemetry_builder::build_packet(
    std::uint32_t timestamp_ms, std::uint32_t flags) noexcept
{
    grid_side_telemetry_packet packet{};

    // Populate header
    packet.header.packet_type = static_cast<std::uint8_t>(packet_type::grid_side_telemetry);
    packet.header.protocol_version = protocol_version;
    packet.header.flags = flags;
    packet.header.timestamp_ms = timestamp_ms;
    packet.header.sequence_number = _sequence_number++;
    packet.header.payload_length = static_cast<std::uint16_t>(calculate_payload_size(flags));

    // Populate requested signal groups
    if (flags & telemetry_flags::measurements) {
        populate_measurements(packet.measurements);
    }

    if (flags & telemetry_flags::observer) {
        populate_observer(packet.observer);
    }

    if (flags & telemetry_flags::modulator) {
        populate_modulator(packet.modulator);
    }

    if (flags & telemetry_flags::controller) {
        populate_controller(packet.controller);
    }

    if (flags & telemetry_flags::state_machine) {
        populate_state_machine(packet.state_machine);
    }

    return packet;
}

grid_side_telemetry_packet grid_side_telemetry_builder::build_full_packet(std::uint32_t timestamp_ms) noexcept
{
    // Build packet with all available signal groups
    std::uint32_t flags = telemetry_flags::measurements | telemetry_flags::observer | telemetry_flags::modulator |
                          telemetry_flags::controller | telemetry_flags::state_machine;

    return build_packet(timestamp_ms, flags);
}

}  // namespace yeet::application::sfc::telemetry
