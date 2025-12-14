#pragma once

#include "yeet_control/edge_detector.hpp"

namespace yeet::application::sfc {

/**
 * @brief Policy controlling when a latched trip may be cleared.
 */
enum class sfc_trip_clear_policy {
    /// Clear latched trip when the supervisor/state-machine acknowledges.
    ack_only,

    /// Clear latched trip only when acknowledged AND the fault condition is no longer active.
    ack_and_fault_clear,
};

/**
 * @brief Shared trip + alarm confirmation handshake used by SFC protection runners.
 *
 * Responsibilities:
 * - Latch `is_tripped` when any protection reports a trip.
 * - Latch `is_inhibited` (alarm pending) whenever a trip has occurred.
 * - Clear `is_tripped` based on a configured policy and the provided acknowledge/fault signals.
 * - Clear `is_inhibited` only on a rising edge of `confirm_alarms` AND only when not tripped.
 *
 * This block is allocation-free and intended for hard real-time usage.
 */
class sfc_protections_trip_handshake {
public:
    /**
     * @brief Configure how a latched trip may be cleared.
     */
    void configure_trip_clear_policy(sfc_trip_clear_policy policy) noexcept;

    /**
     * @brief Reset internal states to a safe initial condition.
     */
    void reset() noexcept;

    /**
     * @brief Execute one cycle of the handshake.
     *
     * @param any_trip_request True if any protection (severity-routed) requests a trip.
     * @param any_fault_condition_active True if the underlying fault condition is still active.
     * @param trip_acknowledged Acknowledge from state machine/supervisor.
     * @param confirm_alarms Operator/supervisor confirmation input (rising edge clears inhibit).
     */
    void update(
        bool any_trip_request, bool any_fault_condition_active, bool trip_acknowledged, bool confirm_alarms) noexcept;

    /**
     * @brief Latched trip output.
     */
    bool get_is_tripped() const noexcept;

    /**
     * @brief Latched inhibit output (alarm pending / confirmation required).
     */
    bool get_is_inhibited() const noexcept;

    /**
     * @brief True for exactly one cycle when the latched trip is cleared in update().
     */
    bool get_trip_cleared_pulse() const noexcept;

private:
    sfc_trip_clear_policy _trip_clear_policy{sfc_trip_clear_policy::ack_only};

    bool _trip_latched{false};
    bool _alarm_pending{false};
    bool _trip_cleared_pulse{false};

    yeet::control::edge_detector _confirm_alarms_edge;
};

}  // namespace yeet::application::sfc
