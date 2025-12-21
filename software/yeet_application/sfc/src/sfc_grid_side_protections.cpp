#include "sfc/sfc_grid_side_protections.hpp"

namespace yeet::application::sfc {

sfc_grid_side_protections::sfc_grid_side_protections(duration_t sampling_time) noexcept
    : _overcurrent_protection(sampling_time),
      _overvoltage_protection(sampling_time),
      _undervoltage_protection(sampling_time),
      _communication_watchdog(sampling_time)
{
}

void sfc_grid_side_protections::reset() noexcept
{
    _overcurrent_protection.reset();
    _overvoltage_protection.reset();
    _undervoltage_protection.reset();
    _communication_watchdog.reset();

    _trip_handshake.reset();

    _is_tripped = false;
    _is_inhibited = false;
    _any_fault_active = false;
    _any_alarm_active = false;

    _overcurrent_tripped = false;
    _overcurrent_alarmed = false;
    _overvoltage_tripped = false;
    _overvoltage_alarmed = false;
    _undervoltage_tripped = false;
    _undervoltage_alarmed = false;

    _overcurrent_warning = false;
    _overvoltage_warning = false;
    _undervoltage_warning = false;

    _communication_loss_tripped = false;
    _communication_loss_alarmed = false;
    _communication_loss_warning = false;

    _door_open_tripped = false;
    _door_open_alarmed = false;
}

void sfc_grid_side_protections::update_overcurrent_protection(current_pu_t dc_link_current_pu) noexcept
{
    _overcurrent_protection.update(dc_link_current_pu);
}

void sfc_grid_side_protections::update_overvoltage_protection(voltage_pu_t grid_voltage_magnitude_pu) noexcept
{
    _overvoltage_protection.update(grid_voltage_magnitude_pu);
}

void sfc_grid_side_protections::update_undervoltage_protection(voltage_pu_t grid_voltage_magnitude_pu) noexcept
{
    _undervoltage_protection.update(grid_voltage_magnitude_pu);
}

void sfc_grid_side_protections::update_communication_watchdog(bool communication_valid) noexcept
{
    _communication_watchdog.update(communication_valid);
}

void sfc_grid_side_protections::resolve_protections(
    bool door_is_open, bool module_enabled, bool confirm_alarms) noexcept
{
    if (!module_enabled) {
        reset();
        return;
    }

    // Reset protections that are ready to be reset (condition cleared + no latched trip)
    if (_overcurrent_protection.can_reset()) {
        _overcurrent_protection.reset();
    }
    if (_overvoltage_protection.can_reset()) {
        _overvoltage_protection.reset();
    }
    if (_undervoltage_protection.can_reset()) {
        _undervoltage_protection.reset();
    }

    const bool overcurrent_trip_raw = _overcurrent_protection.is_tripped();
    const bool overvoltage_trip_raw = _overvoltage_protection.is_tripped();
    const bool undervoltage_trip_raw = _undervoltage_protection.is_tripped();
    const bool comm_loss_trip_raw = _communication_watchdog.is_tripped();
    const bool door_open_raw = door_is_open;

    const bool overcurrent_trip = _overcurrent_is_trip && overcurrent_trip_raw;
    const bool overvoltage_trip = _overvoltage_is_trip && overvoltage_trip_raw;
    const bool undervoltage_trip = _undervoltage_is_trip && undervoltage_trip_raw;
    const bool comm_loss_trip = _comm_loss_is_trip && comm_loss_trip_raw;
    const bool door_open_trip = _door_open_is_trip && door_open_raw;

    const bool overcurrent_alarm = (!_overcurrent_is_trip) && overcurrent_trip_raw;
    const bool overvoltage_alarm = (!_overvoltage_is_trip) && overvoltage_trip_raw;
    const bool undervoltage_alarm = (!_undervoltage_is_trip) && undervoltage_trip_raw;
    const bool comm_loss_alarm = (!_comm_loss_is_trip) && comm_loss_trip_raw;
    const bool door_open_alarm = (!_door_open_is_trip) && door_open_raw;

    _overcurrent_tripped = overcurrent_trip;
    _overcurrent_alarmed = overcurrent_alarm;
    _overvoltage_tripped = overvoltage_trip;
    _overvoltage_alarmed = overvoltage_alarm;
    _undervoltage_tripped = undervoltage_trip;
    _undervoltage_alarmed = undervoltage_alarm;
    _communication_loss_tripped = comm_loss_trip;
    _communication_loss_alarmed = comm_loss_alarm;
    _door_open_tripped = door_open_trip;
    _door_open_alarmed = door_open_alarm;

    _overcurrent_warning = _overcurrent_protection.is_warning();
    _overvoltage_warning = _overvoltage_protection.is_warning();
    _undervoltage_warning = _undervoltage_protection.is_warning();
    _communication_loss_warning = _communication_watchdog.is_warning();

    const bool any_protection_tripped =
        overcurrent_trip || overvoltage_trip || undervoltage_trip || comm_loss_trip || door_open_trip;

    const bool any_fault_active =
        overcurrent_trip_raw || overvoltage_trip_raw || undervoltage_trip_raw || comm_loss_trip_raw || door_open_raw;

    const bool any_alarm_active =
        overcurrent_alarm || overvoltage_alarm || undervoltage_alarm || comm_loss_alarm || door_open_alarm;

    _any_fault_active = any_fault_active;
    _any_alarm_active = any_alarm_active;

    _trip_handshake.update(any_protection_tripped, any_fault_active, confirm_alarms);

    if (_trip_handshake.get_trip_cleared_pulse()) {
        _overcurrent_protection.update(current_pu_t{0.0f});
        _overcurrent_protection.reset();

        _overvoltage_protection.update(voltage_pu_t{0.0f});
        _overvoltage_protection.reset();

        _undervoltage_protection.update(voltage_pu_t{1.0f});
        _undervoltage_protection.reset();

        _communication_watchdog.update(true);
        _communication_watchdog.reset();

        // Clear severity-routed outputs immediately.
        _overcurrent_tripped = false;
        _overcurrent_alarmed = false;
        _overvoltage_tripped = false;
        _overvoltage_alarmed = false;
        _undervoltage_tripped = false;
        _undervoltage_alarmed = false;
        _communication_loss_tripped = false;
        _communication_loss_alarmed = false;
        _door_open_tripped = false;
        _door_open_alarmed = false;
        _any_fault_active = false;
        _any_alarm_active = false;
    }

    _is_tripped = _trip_handshake.get_is_tripped();
    _is_inhibited = _trip_handshake.get_is_inhibited();
}

void sfc_grid_side_protections::configure_overcurrent_is_trip(bool is_trip) noexcept { _overcurrent_is_trip = is_trip; }

void sfc_grid_side_protections::configure_overvoltage_is_trip(bool is_trip) noexcept { _overvoltage_is_trip = is_trip; }

void sfc_grid_side_protections::configure_undervoltage_is_trip(bool is_trip) noexcept
{
    _undervoltage_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_communication_loss_is_trip(bool is_trip) noexcept
{
    _comm_loss_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_door_open_is_trip(bool is_trip) noexcept { _door_open_is_trip = is_trip; }

void sfc_grid_side_protections::configure_overcurrent_trip_threshold(current_pu_t threshold) noexcept
{
    _overcurrent_protection.configure_trip_threshold(threshold);
}

void sfc_grid_side_protections::configure_overcurrent_warning_threshold(current_pu_t threshold) noexcept
{
    _overcurrent_protection.configure_warning_threshold(threshold);
}

void sfc_grid_side_protections::configure_overcurrent_trip_delay(duration_t delay) noexcept
{
    _overcurrent_protection.configure_trip_delay(delay);
}

void sfc_grid_side_protections::configure_overcurrent_warning_delay(duration_t delay) noexcept
{
    _overcurrent_protection.configure_warning_delay(delay);
}

void sfc_grid_side_protections::configure_overvoltage_trip_threshold(voltage_pu_t threshold) noexcept
{
    _overvoltage_protection.configure_trip_threshold(threshold);
}

void sfc_grid_side_protections::configure_overvoltage_warning_threshold(voltage_pu_t threshold) noexcept
{
    _overvoltage_protection.configure_warning_threshold(threshold);
}

void sfc_grid_side_protections::configure_overvoltage_trip_delay(duration_t delay) noexcept
{
    _overvoltage_protection.configure_trip_delay(delay);
}

void sfc_grid_side_protections::configure_overvoltage_warning_delay(duration_t delay) noexcept
{
    _overvoltage_protection.configure_warning_delay(delay);
}

void sfc_grid_side_protections::configure_undervoltage_trip_threshold(voltage_pu_t threshold) noexcept
{
    _undervoltage_protection.configure_trip_threshold(threshold);
}

void sfc_grid_side_protections::configure_undervoltage_warning_threshold(voltage_pu_t threshold) noexcept
{
    _undervoltage_protection.configure_warning_threshold(threshold);
}

void sfc_grid_side_protections::configure_undervoltage_trip_delay(duration_t delay) noexcept
{
    _undervoltage_protection.configure_trip_delay(delay);
}

void sfc_grid_side_protections::configure_undervoltage_warning_delay(duration_t delay) noexcept
{
    _undervoltage_protection.configure_warning_delay(delay);
}

void sfc_grid_side_protections::configure_comm_timeout_duration(duration_t timeout) noexcept
{
    _communication_watchdog.configure_timeout_duration(timeout);
}

void sfc_grid_side_protections::configure_comm_reset_duration(duration_t reset) noexcept
{
    _communication_watchdog.configure_reset_duration(reset);
}

bool sfc_grid_side_protections::get_is_tripped() const noexcept { return _is_tripped; }

bool sfc_grid_side_protections::get_is_inhibited() const noexcept { return _is_inhibited; }

bool sfc_grid_side_protections::get_any_fault_active() const noexcept { return _any_fault_active; }

bool sfc_grid_side_protections::get_any_alarm_active() const noexcept { return _any_alarm_active; }

bool sfc_grid_side_protections::get_overcurrent_tripped() const noexcept { return _overcurrent_tripped; }

bool sfc_grid_side_protections::get_overcurrent_alarmed() const noexcept { return _overcurrent_alarmed; }

bool sfc_grid_side_protections::get_overvoltage_tripped() const noexcept { return _overvoltage_tripped; }

bool sfc_grid_side_protections::get_overvoltage_alarmed() const noexcept { return _overvoltage_alarmed; }

bool sfc_grid_side_protections::get_undervoltage_tripped() const noexcept { return _undervoltage_tripped; }

bool sfc_grid_side_protections::get_undervoltage_alarmed() const noexcept { return _undervoltage_alarmed; }

bool sfc_grid_side_protections::get_overcurrent_warning() const noexcept { return _overcurrent_warning; }

bool sfc_grid_side_protections::get_overvoltage_warning() const noexcept { return _overvoltage_warning; }

bool sfc_grid_side_protections::get_undervoltage_warning() const noexcept { return _undervoltage_warning; }

bool sfc_grid_side_protections::get_communication_loss_tripped() const noexcept { return _communication_loss_tripped; }

bool sfc_grid_side_protections::get_communication_loss_alarmed() const noexcept { return _communication_loss_alarmed; }

bool sfc_grid_side_protections::get_communication_loss_warning() const noexcept { return _communication_loss_warning; }

bool sfc_grid_side_protections::get_door_open_tripped() const noexcept { return _door_open_tripped; }

bool sfc_grid_side_protections::get_door_open_alarmed() const noexcept { return _door_open_alarmed; }

}  // namespace yeet::application::sfc
