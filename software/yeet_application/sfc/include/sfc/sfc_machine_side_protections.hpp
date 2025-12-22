#pragma once

#include "sfc/sfc_communication_watchdog.hpp"
#include "sfc/sfc_protections_trip_handshake.hpp"
#include "sfc/sfc_stall_protection.hpp"
#include "sfc/sfc_vhz_protection.hpp"
#include "yeet_control/threshold_protection.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Machine-side protections orchestration for SFC.
 *
 * This is a reusable, allocation-free component intended to be used by both
 * host-side integration runners and embedded runners.
 *
 * The component owns the underlying protection blocks and implements:
 * - severity routing (trip vs alarm)
 * - aggregate trip + alarm confirmation handshake
 * - consistent output reporting
 */
class sfc_machine_side_protections {
public:
    explicit sfc_machine_side_protections(duration_t sampling_time) noexcept;

    void reset() noexcept;

    void resolve_protections(bool module_enabled, bool confirm_alarms) noexcept;

    // -------------------- Individual Protection Updates --------------------

    void update_vhz_protection(voltage_pu_t machine_voltage_magnitude_pu,
        frequency_pu_t machine_frequency_pu,
        bool closed_loop_active) noexcept;

    void update_stall_protection(
        current_pu_t dc_link_current_pu, speed_pu_t machine_speed_pu, bool startup_active) noexcept;

    void update_overspeed_protection(speed_pu_t machine_speed_pu, bool closed_loop_active) noexcept;

    void update_machine_overvoltage_protection(voltage_pu_t machine_voltage_magnitude_pu) noexcept;

    void update_loss_of_excitation_protection(
        bool excitation_is_on, bool startup_active, bool closed_loop_active) noexcept;

    void update_communication_watchdog(bool communication_valid) noexcept;

    void update_external_fault(bool active) noexcept;

    // -------------------- Configuration (severity routing) --------------------

    void configure_over_flux_is_trip(bool is_trip) noexcept;
    void configure_under_flux_is_trip(bool is_trip) noexcept;
    void configure_stall_is_trip(bool is_trip) noexcept;
    void configure_overspeed_is_trip(bool is_trip) noexcept;
    void configure_machine_overvoltage_is_trip(bool is_trip) noexcept;
    void configure_communication_loss_is_trip(bool is_trip) noexcept;
    void configure_loss_of_excitation_is_trip(bool is_trip) noexcept;
    void configure_external_fault_is_trip(bool is_trip) noexcept;

    // -------------------- Configuration (per-protection parameters) --------------------

    // V/Hz protection
    void configure_over_flux_trip_threshold(flux_pu_t threshold) noexcept;
    void configure_over_flux_warning_threshold(flux_pu_t threshold) noexcept;
    void configure_over_flux_trip_delay(duration_t delay) noexcept;
    void configure_over_flux_warning_delay(duration_t delay) noexcept;

    void configure_under_flux_trip_threshold(flux_pu_t threshold) noexcept;
    void configure_under_flux_warning_threshold(flux_pu_t threshold) noexcept;
    void configure_under_flux_trip_delay(duration_t delay) noexcept;
    void configure_under_flux_warning_delay(duration_t delay) noexcept;

    // Stall protection
    void configure_stall_current_threshold(current_pu_t threshold) noexcept;
    void configure_stall_speed_threshold(speed_pu_t threshold) noexcept;
    void configure_stall_trip_delay(duration_t delay) noexcept;
    void configure_stall_warning_delay(duration_t delay) noexcept;

    // Communication watchdog
    void configure_comm_timeout_duration(duration_t timeout) noexcept;
    void configure_comm_reset_duration(duration_t reset) noexcept;

    // Overspeed protection
    void configure_overspeed_trip_threshold(speed_pu_t threshold) noexcept;
    void configure_overspeed_trip_delay(duration_t delay) noexcept;

    // Machine overvoltage protection
    void configure_machine_overvoltage_trip_threshold(voltage_pu_t threshold) noexcept;
    void configure_machine_overvoltage_trip_delay(duration_t delay) noexcept;

    // Loss of excitation protection
    void configure_loss_of_excitation_trip_delay(duration_t delay) noexcept;

    // -------------------- Outputs --------------------

    bool get_is_tripped() const noexcept;
    bool get_is_inhibited() const noexcept;
    bool get_any_fault_active() const noexcept;
    bool get_any_alarm_active() const noexcept;

    bool get_over_flux_tripped() const noexcept;
    bool get_over_flux_alarmed() const noexcept;
    bool get_under_flux_tripped() const noexcept;
    bool get_under_flux_alarmed() const noexcept;

    bool get_stall_tripped() const noexcept;
    bool get_stall_alarmed() const noexcept;

    bool get_overspeed_tripped() const noexcept;
    bool get_overspeed_alarmed() const noexcept;

    bool get_machine_overvoltage_tripped() const noexcept;
    bool get_machine_overvoltage_alarmed() const noexcept;

    bool get_communication_loss_tripped() const noexcept;
    bool get_communication_loss_alarmed() const noexcept;

    bool get_loss_of_excitation_tripped() const noexcept;
    bool get_loss_of_excitation_alarmed() const noexcept;

    bool get_external_fault_tripped() const noexcept;
    bool get_external_fault_alarmed() const noexcept;

    bool get_emergency_off_tripped() const noexcept;
    bool get_emergency_off_alarmed() const noexcept;

    bool get_over_flux_warning() const noexcept;
    bool get_under_flux_warning() const noexcept;
    bool get_stall_warning() const noexcept;
    bool get_communication_loss_warning() const noexcept;

    flux_pu_t get_vhz_ratio_pu() const noexcept;

private:
    // Owned protection blocks
    sfc_vhz_protection _vhz_protection;
    sfc_stall_protection _stall_protection;
    sfc_communication_watchdog _communication_watchdog;

    yeet::control::threshold_protection<speed_pu_t, yeet::control::threshold_direction::above> _overspeed_protection;
    yeet::control::threshold_protection<voltage_pu_t, yeet::control::threshold_direction::above>
        _machine_overvoltage_protection;
    yeet::control::threshold_protection<real_t, yeet::control::threshold_direction::below>
        _loss_of_excitation_protection;

    sfc_protections_trip_handshake _trip_handshake;

    bool _external_fault_active{false};

    // Severity routing configuration
    bool _over_flux_is_trip{true};
    bool _under_flux_is_trip{true};
    bool _stall_is_trip{true};
    bool _overspeed_is_trip{true};
    bool _machine_overvoltage_is_trip{true};
    bool _comm_loss_is_trip{true};
    bool _loss_of_excitation_is_trip{true};
    bool _external_fault_is_trip{true};

    // Cache of thresholds for present-time fault condition detection
    flux_pu_t _over_flux_trip_threshold{flux_pu_t{1.10f}};
    flux_pu_t _under_flux_trip_threshold{flux_pu_t{0.80f}};
    speed_pu_t _overspeed_trip_threshold{speed_pu_t{1.10f}};
    voltage_pu_t _machine_overvoltage_trip_threshold{voltage_pu_t{1.10f}};

    // Outputs (latched/derived)
    bool _is_tripped{false};
    bool _is_inhibited{false};
    bool _any_fault_active{false};
    bool _any_alarm_active{false};

    bool _over_flux_tripped{false};
    bool _over_flux_alarmed{false};
    bool _under_flux_tripped{false};
    bool _under_flux_alarmed{false};

    bool _stall_tripped{false};
    bool _stall_alarmed{false};

    bool _overspeed_tripped{false};
    bool _overspeed_alarmed{false};

    bool _machine_overvoltage_tripped{false};
    bool _machine_overvoltage_alarmed{false};

    bool _communication_loss_tripped{false};
    bool _communication_loss_alarmed{false};

    bool _loss_of_excitation_tripped{false};
    bool _loss_of_excitation_alarmed{false};

    bool _external_fault_tripped{false};
    bool _external_fault_alarmed{false};

    bool _over_flux_warning{false};
    bool _under_flux_warning{false};
    bool _stall_warning{false};
    bool _communication_loss_warning{false};
};

}  // namespace yeet::application::sfc
