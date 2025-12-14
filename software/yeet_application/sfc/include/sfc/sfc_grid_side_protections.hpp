#pragma once

#include "sfc/sfc_communication_watchdog.hpp"
#include "sfc/sfc_current_protection.hpp"
#include "sfc/sfc_overvoltage_protection.hpp"
#include "sfc/sfc_protections_trip_handshake.hpp"
#include "sfc/sfc_undervoltage_protection.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Grid-side protections orchestration for SFC.
 *
 * This is a reusable, allocation-free component intended to be used by both
 * host-side integration runners and embedded runners.
 *
 * The component owns the underlying protection blocks and implements:
 * - severity routing (trip vs alarm)
 * - aggregate trip + alarm confirmation handshake
 * - consistent output reporting
 */
class sfc_grid_side_protections {
public:
    explicit sfc_grid_side_protections(duration_t sampling_time) noexcept;

    void reset() noexcept;

    void update(current_pu_t dc_link_current_pu,
        voltage_pu_t grid_voltage_magnitude_pu,
        bool door_is_open,
        bool module_enabled,
        bool release_protections,
        bool communication_valid,
        bool trip_acknowledged,
        bool confirm_alarms) noexcept;

    // -------------------- Configuration (severity routing) --------------------

    void configure_overcurrent_is_trip(bool is_trip) noexcept;
    void configure_overvoltage_is_trip(bool is_trip) noexcept;
    void configure_undervoltage_is_trip(bool is_trip) noexcept;
    void configure_communication_loss_is_trip(bool is_trip) noexcept;
    void configure_door_open_is_trip(bool is_trip) noexcept;

    // -------------------- Configuration (per-protection parameters) --------------------

    // Overcurrent
    void configure_overcurrent_trip_threshold(current_pu_t threshold) noexcept;
    void configure_overcurrent_warning_threshold(current_pu_t threshold) noexcept;
    void configure_overcurrent_trip_delay(duration_t delay) noexcept;
    void configure_overcurrent_warning_delay(duration_t delay) noexcept;

    // Overvoltage
    void configure_overvoltage_trip_threshold(voltage_pu_t threshold) noexcept;
    void configure_overvoltage_warning_threshold(voltage_pu_t threshold) noexcept;
    void configure_overvoltage_trip_delay(duration_t delay) noexcept;
    void configure_overvoltage_warning_delay(duration_t delay) noexcept;

    // Undervoltage
    void configure_undervoltage_trip_threshold(voltage_pu_t threshold) noexcept;
    void configure_undervoltage_warning_threshold(voltage_pu_t threshold) noexcept;
    void configure_undervoltage_trip_delay(duration_t delay) noexcept;
    void configure_undervoltage_warning_delay(duration_t delay) noexcept;

    // Communication watchdog
    void configure_comm_timeout_duration(duration_t timeout) noexcept;
    void configure_comm_reset_duration(duration_t reset) noexcept;

    // -------------------- Outputs --------------------

    bool get_is_tripped() const noexcept;
    bool get_is_inhibited() const noexcept;
    bool get_any_fault_active() const noexcept;

    bool get_overcurrent_tripped() const noexcept;
    bool get_overcurrent_alarmed() const noexcept;
    bool get_overvoltage_tripped() const noexcept;
    bool get_overvoltage_alarmed() const noexcept;
    bool get_undervoltage_tripped() const noexcept;
    bool get_undervoltage_alarmed() const noexcept;

    bool get_overcurrent_warning() const noexcept;
    bool get_overvoltage_warning() const noexcept;
    bool get_undervoltage_warning() const noexcept;

    bool get_communication_loss_tripped() const noexcept;
    bool get_communication_loss_alarmed() const noexcept;
    bool get_communication_loss_warning() const noexcept;

    bool get_door_open_tripped() const noexcept;
    bool get_door_open_alarmed() const noexcept;

private:
    // Owned protection blocks
    sfc_current_protection _overcurrent_protection;
    sfc_overvoltage_protection _overvoltage_protection;
    sfc_undervoltage_protection _undervoltage_protection;
    sfc_communication_watchdog _communication_watchdog;

    sfc_protections_trip_handshake _trip_handshake;

    // Severity routing configuration
    bool _overcurrent_is_trip{true};
    bool _overvoltage_is_trip{true};
    bool _undervoltage_is_trip{true};
    bool _comm_loss_is_trip{true};
    bool _door_open_is_trip{true};

    // Outputs (latched/derived)
    bool _is_tripped{false};
    bool _is_inhibited{false};
    bool _any_fault_active{false};

    bool _overcurrent_tripped{false};
    bool _overcurrent_alarmed{false};
    bool _overvoltage_tripped{false};
    bool _overvoltage_alarmed{false};
    bool _undervoltage_tripped{false};
    bool _undervoltage_alarmed{false};

    bool _overcurrent_warning{false};
    bool _overvoltage_warning{false};
    bool _undervoltage_warning{false};

    bool _communication_loss_tripped{false};
    bool _communication_loss_alarmed{false};
    bool _communication_loss_warning{false};

    bool _door_open_tripped{false};
    bool _door_open_alarmed{false};
};

}  // namespace yeet::application::sfc
