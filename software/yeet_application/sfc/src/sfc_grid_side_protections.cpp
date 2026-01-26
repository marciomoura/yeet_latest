#include "sfc/sfc_grid_side_protections.hpp"

namespace yeet::application::sfc {

sfc_grid_side_protections::sfc_grid_side_protections(duration_t sampling_time) noexcept
    : _overcurrent_protection(sampling_time),
      _overvoltage_protection(sampling_time),
      _undervoltage_protection(sampling_time),
      _communication_watchdog(sampling_time),
      _is_confirm_trip_rising_edge_detector(),
      _emergency_on_off_reset_timer((double)sampling_time.value())
{
    _emergency_on_off_reset_timer.configure(1.0f, 1.0f);
}

void sfc_grid_side_protections::reset() noexcept
{
    _overcurrent_protection.reset();
    _overvoltage_protection.reset();
    _undervoltage_protection.reset();
    _communication_watchdog.reset();

    _trip_handshake.reset();
    _is_confirm_trip_rising_edge_detector.reset();

    _emergency_off_edge_detected = {};
    _emergency_on_off_reset_timer.reset();

    _external_fault_active = false;
    _emergency_off_active = false;
    _emergency_off_reset = false;

    _has_active_trip = false;
    _has_active_alarm = false;
    _is_inhibited = false;
    _first_trip_source = grid_trip_source::none;

    _overcurrent_tripped = false;
    _overcurrent_alarmed = false;
    _overvoltage_tripped = false;
    _overvoltage_alarmed = false;
    _undervoltage_tripped = false;
    _undervoltage_alarmed = false;
    _undervoltage_detected = false;

    _overcurrent_warning = false;
    _overvoltage_warning = false;
    _undervoltage_warning = false;

    _communication_loss_tripped = false;
    _communication_loss_alarmed = false;
    _communication_loss_warning = false;

    _door_open_tripped = false;
    _door_open_alarmed = false;

    _external_fault_tripped = false;
    _external_fault_alarmed = false;

    _emergency_off_tripped = false;
    _emergency_off_alarmed = false;
}

void sfc_grid_side_protections::update_overcurrent_protection(current_pu_t dc_link_current_pu) noexcept
{
    _overcurrent_protection.update(dc_link_current_pu);
}

void sfc_grid_side_protections::update_overvoltage_protection(voltage_pu_t grid_voltage_magnitude_pu) noexcept
{
    _overvoltage_protection.update(grid_voltage_magnitude_pu);
}

void sfc_grid_side_protections::update_undervoltage_protection(
    voltage_pu_t grid_voltage_magnitude_pu, bool enabled) noexcept
{
    _undervoltage_protection.update(grid_voltage_magnitude_pu, enabled);
}

void sfc_grid_side_protections::update_communication_watchdog(bool communication_valid) noexcept
{
    _communication_watchdog.update(communication_valid);
}

void sfc_grid_side_protections::update_external_fault(bool active) noexcept { _external_fault_active = active; }

void sfc_grid_side_protections::update_emergency_off(bool active) noexcept { _emergency_off_active = active; }

void sfc_grid_side_protections::update_machine_protection_tripped(bool tripped) noexcept
{
    _machine_protection_tripped_active = tripped;
}

void sfc_grid_side_protections::resolve_protections(bool door_is_open, bool module_enabled, bool confirm_trips) noexcept
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
    const bool external_fault_raw = _external_fault_active;
    const bool emergency_off_raw = _emergency_off_active;
    const bool machine_protection_raw = _machine_protection_tripped_active;

    const bool overcurrent_trip = _overcurrent_is_trip && overcurrent_trip_raw;
    const bool overvoltage_trip = _overvoltage_is_trip && overvoltage_trip_raw;
    const bool undervoltage_trip = _undervoltage_is_trip && undervoltage_trip_raw;
    const bool comm_loss_trip = _comm_loss_is_trip && comm_loss_trip_raw;
    const bool door_open_trip = _door_open_is_trip && door_open_raw;
    const bool external_fault_trip = _external_fault_is_trip && external_fault_raw;
    const bool emergency_off_trip = _emergency_off_is_trip && emergency_off_raw;
    const bool machine_protection_trip = _machine_protection_is_trip && machine_protection_raw;

    const bool overcurrent_alarm = (!_overcurrent_is_trip) && overcurrent_trip_raw;
    const bool overvoltage_alarm = (!_overvoltage_is_trip) && overvoltage_trip_raw;
    const bool undervoltage_alarm = (!_undervoltage_is_trip) && undervoltage_trip_raw;
    const bool comm_loss_alarm = (!_comm_loss_is_trip) && comm_loss_trip_raw;
    const bool door_open_alarm = (!_door_open_is_trip) && door_open_raw;
    const bool external_fault_alarm = (!_external_fault_is_trip) && external_fault_raw;
    const bool emergency_off_alarm = (!_emergency_off_is_trip) && emergency_off_raw;
    const bool machine_protection_alarm = (!_machine_protection_is_trip) && machine_protection_raw;

    _overcurrent_tripped = overcurrent_trip;
    _overcurrent_alarmed = overcurrent_alarm;
    _overvoltage_tripped = overvoltage_trip;
    _overvoltage_alarmed = overvoltage_alarm;
    _undervoltage_tripped = undervoltage_trip;
    _undervoltage_alarmed = undervoltage_alarm;
    _undervoltage_detected = _undervoltage_protection.is_detected();
    _communication_loss_tripped = comm_loss_trip;
    _communication_loss_alarmed = comm_loss_alarm;
    _door_open_tripped = door_open_trip;
    _door_open_alarmed = door_open_alarm;
    _external_fault_tripped = external_fault_trip;
    _external_fault_alarmed = external_fault_alarm;
    _emergency_off_tripped = emergency_off_trip;
    _emergency_off_alarmed = emergency_off_alarm;
    _machine_protection_tripped = machine_protection_trip;
    _machine_protection_alarmed = machine_protection_alarm;

    _overcurrent_warning = _overcurrent_protection.is_warning();
    _overvoltage_warning = _overvoltage_protection.is_warning();
    _undervoltage_warning = _undervoltage_protection.is_warning();
    _communication_loss_warning = _communication_watchdog.is_warning();

    const bool any_protection_tripped = overcurrent_trip || overvoltage_trip || undervoltage_trip || comm_loss_trip ||
                                        door_open_trip || external_fault_trip || emergency_off_trip ||
                                        machine_protection_trip;

    // Latch first trip source
    if (_first_trip_source == grid_trip_source::none && any_protection_tripped) {
        if (overcurrent_trip) {
            _first_trip_source = grid_trip_source::overcurrent;
        }
        else if (overvoltage_trip) {
            _first_trip_source = grid_trip_source::overvoltage;
        }
        else if (undervoltage_trip) {
            _first_trip_source = grid_trip_source::undervoltage;
        }
        else if (comm_loss_trip) {
            _first_trip_source = grid_trip_source::communication_loss;
        }
        else if (door_open_trip) {
            _first_trip_source = grid_trip_source::door_open;
        }
        else if (external_fault_trip) {
            _first_trip_source = grid_trip_source::external_fault;
        }
        else if (emergency_off_trip) {
            _first_trip_source = grid_trip_source::emergency_off;
        }
        else if (machine_protection_trip) {
            _first_trip_source = grid_trip_source::machine_protection_feedback;
        }
    }

    const bool any_alarm_active = overcurrent_alarm || overvoltage_alarm || undervoltage_alarm || comm_loss_alarm ||
                                  door_open_alarm || external_fault_alarm || emergency_off_alarm ||
                                  machine_protection_alarm;

    // Update the handshake with current trip state
    _trip_handshake.update(any_protection_tripped, confirm_trips);

    // has_active_trip reflects real-time state (clears automatically when fault clears)
    _has_active_trip = any_protection_tripped;
    _has_active_alarm = any_alarm_active;

    _is_inhibited = _trip_handshake.get_is_inhibited();

    // The reset pulse is triggered when the confirm_trips signal goes true (trip cleared and confirmed).
    _is_confirm_trip_rising_edge_detector.update(confirm_trips);
    const bool reset_trigger = _is_confirm_trip_rising_edge_detector.is_rising_edge();

    if (reset_trigger && !_emergency_off_edge_detected) {
        _emergency_off_edge_detected = true;
    }

    _emergency_on_off_reset_timer.update(_emergency_off_edge_detected);
    // Now it needs to go to false
    if (_emergency_on_off_reset_timer.get_output()) {
        _emergency_off_edge_detected = false;
    }

    _emergency_off_reset = _emergency_on_off_reset_timer.get_output();
}

void sfc_grid_side_protections::check_and_reset_handshake_if_no_trips() noexcept
{
    // Check if ANY trip is currently active (after severity routing)
    const bool overcurrent_trip = _overcurrent_is_trip && _overcurrent_protection.is_tripped();
    const bool overvoltage_trip = _overvoltage_is_trip && _overvoltage_protection.is_tripped();
    const bool undervoltage_trip = _undervoltage_is_trip && _undervoltage_protection.is_tripped();
    const bool comm_loss_trip = _comm_loss_is_trip && _communication_watchdog.is_tripped();
    const bool door_open_trip = _door_open_is_trip && false;  // door state not stored
    const bool external_fault_trip = _external_fault_is_trip && _external_fault_active;
    const bool emergency_off_trip = _emergency_off_is_trip && _emergency_off_active;
    const bool machine_protection_trip = _machine_protection_is_trip && _machine_protection_tripped_active;

    const bool any_trip_active = overcurrent_trip || overvoltage_trip || undervoltage_trip || comm_loss_trip ||
                                 door_open_trip || external_fault_trip || emergency_off_trip || machine_protection_trip;

    // If no trips are active after severity change, reset the handshake to clear inhibit
    if (!any_trip_active) {
        _trip_handshake.reset();
        _is_inhibited = false;
    }
}

void sfc_grid_side_protections::configure_overcurrent_is_trip(bool is_trip) noexcept
{
    if (_overcurrent_is_trip != is_trip) {
        _overcurrent_protection.reset();
        _overcurrent_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _overcurrent_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_overvoltage_is_trip(bool is_trip) noexcept
{
    if (_overvoltage_is_trip != is_trip) {
        _overvoltage_protection.reset();
        _overvoltage_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _overvoltage_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_undervoltage_is_trip(bool is_trip) noexcept
{
    if (_undervoltage_is_trip != is_trip) {
        _undervoltage_protection.reset();
        _undervoltage_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _undervoltage_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_communication_loss_is_trip(bool is_trip) noexcept
{
    if (_comm_loss_is_trip != is_trip) {
        _communication_watchdog.reset();
        _comm_loss_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _comm_loss_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_door_open_is_trip(bool is_trip) noexcept { _door_open_is_trip = is_trip; }

void sfc_grid_side_protections::configure_external_fault_is_trip(bool is_trip) noexcept
{
    _external_fault_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_emergency_off_is_trip(bool is_trip) noexcept
{
    _emergency_off_is_trip = is_trip;
}

void sfc_grid_side_protections::configure_machine_protection_is_trip(bool is_trip) noexcept
{
    _machine_protection_is_trip = is_trip;
}

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

void sfc_grid_side_protections::configure_emergency_off_reset_duration(duration_t duration) noexcept
{
    _emergency_off_reset_duration = duration;
    _emergency_on_off_reset_timer.configure(1.0f, (real_t)duration.value());
}

bool sfc_grid_side_protections::get_has_active_trip() const noexcept { return _has_active_trip; }

bool sfc_grid_side_protections::get_has_active_alarm() const noexcept { return _has_active_alarm; }

bool sfc_grid_side_protections::get_is_inhibited() const noexcept { return _is_inhibited; }

bool sfc_grid_side_protections::get_overcurrent_tripped() const noexcept { return _overcurrent_tripped; }

bool sfc_grid_side_protections::get_overcurrent_alarmed() const noexcept { return _overcurrent_alarmed; }

bool sfc_grid_side_protections::get_overvoltage_tripped() const noexcept { return _overvoltage_tripped; }

bool sfc_grid_side_protections::get_overvoltage_alarmed() const noexcept { return _overvoltage_alarmed; }

bool sfc_grid_side_protections::get_undervoltage_tripped() const noexcept { return _undervoltage_tripped; }

bool sfc_grid_side_protections::get_undervoltage_alarmed() const noexcept { return _undervoltage_alarmed; }

bool sfc_grid_side_protections::get_undervoltage_detected() const noexcept { return _undervoltage_detected; }

bool sfc_grid_side_protections::get_overcurrent_warning() const noexcept { return _overcurrent_warning; }

bool sfc_grid_side_protections::get_overvoltage_warning() const noexcept { return _overvoltage_warning; }

bool sfc_grid_side_protections::get_undervoltage_warning() const noexcept { return _undervoltage_warning; }

bool sfc_grid_side_protections::get_communication_loss_tripped() const noexcept { return _communication_loss_tripped; }

bool sfc_grid_side_protections::get_communication_loss_alarmed() const noexcept { return _communication_loss_alarmed; }

bool sfc_grid_side_protections::get_communication_loss_warning() const noexcept { return _communication_loss_warning; }

bool sfc_grid_side_protections::get_door_open_tripped() const noexcept { return _door_open_tripped; }

bool sfc_grid_side_protections::get_door_open_alarmed() const noexcept { return _door_open_alarmed; }

bool sfc_grid_side_protections::get_external_fault_tripped() const noexcept { return _external_fault_tripped; }

bool sfc_grid_side_protections::get_external_fault_alarmed() const noexcept { return _external_fault_alarmed; }

bool sfc_grid_side_protections::get_emergency_off_tripped() const noexcept { return _emergency_off_tripped; }

bool sfc_grid_side_protections::get_emergency_off_alarmed() const noexcept { return _emergency_off_alarmed; }
bool sfc_grid_side_protections::get_emergency_off_reset() const noexcept { return _emergency_off_reset; }
bool sfc_grid_side_protections::get_machine_protection_tripped() const noexcept { return _machine_protection_tripped; }

bool sfc_grid_side_protections::get_machine_protection_alarmed() const noexcept { return _machine_protection_alarmed; }

}  // namespace yeet::application::sfc
