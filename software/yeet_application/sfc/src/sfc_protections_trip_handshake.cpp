#include "sfc/sfc_protections_trip_handshake.hpp"

namespace yeet::application::sfc {

void sfc_protections_trip_handshake::reset() noexcept
{
    _trip_latched = false;
    _alarm_pending = false;
    _trip_cleared_pulse = false;

    // No explicit reset on edge detector; a reset is equivalent to forgetting history.
    _confirm_alarms_edge = yeet::control::edge_detector{};
}

void sfc_protections_trip_handshake::update(
    bool any_trip_request, bool any_fault_condition_active, bool confirm_alarms) noexcept
{
    _trip_cleared_pulse = false;

    // Latch trip and alarm_pending when a trip is requested
    if (any_trip_request) {
        _trip_latched = true;
        _alarm_pending = true;
    }

    const bool confirm_edge = _confirm_alarms_edge.update(confirm_alarms);

    // Trip latch clears only when acknowledged AND fault is gone
    if (_trip_latched && confirm_edge && !any_fault_condition_active) {
        _trip_latched = false;
        _trip_cleared_pulse = true;
    }

    // Alarm confirmation (user acknowledgment): rising edge on confirm_alarms clears inhibit
    // ONLY when no active fault conditions exist (trip must be cleared first)
    if (confirm_edge && !_trip_latched && !any_fault_condition_active) {
        _alarm_pending = false;
    }
}

bool sfc_protections_trip_handshake::get_is_tripped() const noexcept { return _trip_latched; }

bool sfc_protections_trip_handshake::get_is_inhibited() const noexcept { return _alarm_pending; }

bool sfc_protections_trip_handshake::get_trip_cleared_pulse() const noexcept { return _trip_cleared_pulse; }

}  // namespace yeet::application::sfc
