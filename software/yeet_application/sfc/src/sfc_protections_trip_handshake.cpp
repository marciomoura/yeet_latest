#include "sfc/sfc_protections_trip_handshake.hpp"

namespace yeet::application::sfc {

void sfc_protections_trip_handshake::configure_trip_clear_policy(sfc_trip_clear_policy policy) noexcept
{
    _trip_clear_policy = policy;
}

void sfc_protections_trip_handshake::reset() noexcept
{
    _trip_latched = false;
    _alarm_pending = false;
    _trip_cleared_pulse = false;

    // No explicit reset on edge detector; a reset is equivalent to forgetting history.
    _confirm_alarms_edge = yeet::control::edge_detector{};
}

void sfc_protections_trip_handshake::update(
    bool any_trip_request, bool any_fault_condition_active, bool trip_acknowledged, bool confirm_alarms) noexcept
{
    _trip_cleared_pulse = false;

    // Latch trip and alarm_pending when a trip is requested
    if (any_trip_request) {
        _trip_latched = true;
        _alarm_pending = true;
    }

    bool can_clear_trip = false;
    if (_alarm_pending) {
        if (_trip_clear_policy == sfc_trip_clear_policy::ack_only) {
            can_clear_trip = trip_acknowledged;
        }
        else {
            can_clear_trip = trip_acknowledged && !any_fault_condition_active;
        }
    }
    else {
        // Prior to any trip occurring, allow the latched output to remain clear.
        // This branch matches existing runner behavior where a non-pending alarm
        // implies no latched shutdown state.
        can_clear_trip = !any_trip_request;
    }

    if (can_clear_trip && _trip_latched) {
        _trip_latched = false;
        _trip_cleared_pulse = true;
    }

    const bool confirm_edge = _confirm_alarms_edge.update(confirm_alarms);

    // Only allow alarm confirmation once trip is cleared.
    if (confirm_edge && !_trip_latched) {
        _alarm_pending = false;
    }
}

bool sfc_protections_trip_handshake::get_is_tripped() const noexcept { return _trip_latched; }

bool sfc_protections_trip_handshake::get_is_inhibited() const noexcept { return _alarm_pending; }

bool sfc_protections_trip_handshake::get_trip_cleared_pulse() const noexcept { return _trip_cleared_pulse; }

}  // namespace yeet::application::sfc
