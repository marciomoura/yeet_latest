/**
 * @file sfc_communication_watchdog.cpp
 * @brief Communication link watchdog implementation
 */

#include "sfc/sfc_communication_watchdog.hpp"

#include <algorithm>

namespace yeet::application::sfc {

sfc_communication_watchdog::sfc_communication_watchdog(
    yeet::duration_t sampling_period, yeet::duration_t timeout_duration, yeet::duration_t reset_duration) noexcept
    : _sampling_period(sampling_period), _invalid_count(0), _is_tripped(false)
{
    // Calculate cycle thresholds from time durations
    // Ensure minimum of 1 cycle for each threshold
    const float timeout_cycles_float = timeout_duration.value() / sampling_period.value();
    const float reset_cycles_float = reset_duration.value() / sampling_period.value();

    _timeout_threshold = std::max(std::uint16_t{1}, static_cast<std::uint16_t>(timeout_cycles_float));
    _reset_threshold =
        std::min(_timeout_threshold, std::max(std::uint16_t{1}, static_cast<std::uint16_t>(reset_cycles_float)));
}

void sfc_communication_watchdog::configure_timeout_cycles(std::uint16_t cycles) noexcept
{
    // Enforce minimum threshold of 1 cycle
    _timeout_threshold = (cycles > 0) ? cycles : 1;

    // Ensure reset threshold is not greater than timeout threshold
    if (_reset_threshold > _timeout_threshold) {
        _reset_threshold = _timeout_threshold;
    }
}

void sfc_communication_watchdog::configure_reset_cycles(std::uint16_t cycles) noexcept
{
    // Enforce minimum threshold of 1 cycle and cap at timeout threshold
    const std::uint16_t requested_cycles = (cycles > 0) ? cycles : 1;
    _reset_threshold = std::min(requested_cycles, _timeout_threshold);
}

void sfc_communication_watchdog::configure_timeout_duration(yeet::duration_t duration) noexcept
{
    // Convert duration to cycles based on sampling period
    const float cycles_float = duration.value() / _sampling_period.value();
    const std::uint16_t cycles = std::max(std::uint16_t{1}, static_cast<std::uint16_t>(cycles_float));
    configure_timeout_cycles(cycles);
}

void sfc_communication_watchdog::configure_reset_duration(yeet::duration_t duration) noexcept
{
    // Convert duration to cycles based on sampling period
    const float cycles_float = duration.value() / _sampling_period.value();
    const std::uint16_t cycles = std::max(std::uint16_t{1}, static_cast<std::uint16_t>(cycles_float));
    configure_reset_cycles(cycles);
}

void sfc_communication_watchdog::update(bool data_valid) noexcept
{
    if (data_valid) {
        // Valid communication received - decrement invalid counter (saturate at 0)
        if (_invalid_count > 0) {
            _invalid_count--;
        }

        // Auto-reset if tripped and counter falls below reset threshold
        if (_is_tripped && _invalid_count <= _reset_threshold) {
            _is_tripped = false;
        }
    }
    else {
        // Invalid communication - increment invalid counter (saturate at timeout threshold)
        if (_invalid_count < _timeout_threshold) {
            _invalid_count++;
        }

        // Trip if counter exceeds threshold
        if (_invalid_count >= _timeout_threshold) {
            _is_tripped = true;
        }
    }
}

void sfc_communication_watchdog::reset() noexcept
{
    _invalid_count = 0;
    _is_tripped = false;
}

bool sfc_communication_watchdog::is_tripped() const noexcept { return _is_tripped; }

bool sfc_communication_watchdog::is_warning() const noexcept
{
    // Warning state: invalid count > 50% of trip threshold
    return _invalid_count > (_timeout_threshold / 2);
}

std::uint16_t sfc_communication_watchdog::get_invalid_count() const noexcept { return _invalid_count; }

std::uint16_t sfc_communication_watchdog::get_timeout_threshold() const noexcept { return _timeout_threshold; }

std::uint16_t sfc_communication_watchdog::get_reset_threshold() const noexcept { return _reset_threshold; }

}  // namespace yeet::application::sfc
