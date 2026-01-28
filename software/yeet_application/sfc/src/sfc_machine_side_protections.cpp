#include "sfc/sfc_machine_side_protections.hpp"

namespace yeet::application::sfc {

sfc_machine_side_protections::sfc_machine_side_protections(duration_t sampling_time) noexcept
    : _vhz_protection(sampling_time),
      _stall_protection(sampling_time),
      _communication_watchdog(sampling_time),
      _overspeed_protection(sampling_time),
      _machine_overvoltage_protection(sampling_time),
      _loss_of_excitation_protection(sampling_time),
      _external_fault_delay(0.0, sampling_time.value())
{
    // Configure threshold-based protections with safe defaults.
    // Without configuration, threshold_protection defaults to zero thresholds which
    // causes immediate trips for any positive-valued measurement.
    configure_over_flux_trip_threshold(flux_pu_t{10.0f});
    configure_over_flux_trip_delay(duration_t{0.0});
    configure_under_flux_trip_threshold(flux_pu_t{0.0f});
    configure_under_flux_trip_delay(duration_t{0.0});

    configure_stall_current_threshold(current_pu_t{10.0f});
    configure_stall_speed_threshold(speed_pu_t{10.0f});
    configure_stall_trip_delay(duration_t{0.0});

    configure_overspeed_trip_threshold(_overspeed_trip_threshold);
    configure_overspeed_trip_delay(duration_t{0.0});

    configure_machine_overvoltage_trip_threshold(_machine_overvoltage_trip_threshold);
    configure_machine_overvoltage_trip_delay(duration_t{0.0});

    // Loss of excitation: Threshold below for detecting excitation_is_on signal going low
    // We treat excitation_is_on as a real_t (1.0 = on, 0.0 = off) and trip if it goes below 0.5
    _loss_of_excitation_protection.configure_trip_threshold(real_t{0.5f});
    _loss_of_excitation_protection.configure_warning_threshold(real_t{0.5f});
    configure_loss_of_excitation_trip_delay(duration_t{0.0});
}

void sfc_machine_side_protections::reset() noexcept
{
    _vhz_protection.reset();
    _stall_protection.reset();
    _communication_watchdog.reset();
    _overspeed_protection.reset();
    _machine_overvoltage_protection.reset();
    _loss_of_excitation_protection.reset();
    _external_fault_delay.reset();
    _trip_handshake.reset();
    _is_confirm_trip_rising_edge_detector.reset();

    _external_fault_active = false;

    _has_active_trip = false;
    _has_active_alarm = false;
    _is_inhibited = false;
    _first_trip_source = machine_trip_source::none;

    _over_flux_tripped = false;
    _over_flux_alarmed = false;
    _under_flux_tripped = false;
    _under_flux_alarmed = false;

    _stall_tripped = false;
    _stall_alarmed = false;

    _overspeed_tripped = false;
    _overspeed_alarmed = false;

    _machine_overvoltage_tripped = false;
    _machine_overvoltage_alarmed = false;

    _communication_loss_tripped = false;
    _communication_loss_alarmed = false;

    _loss_of_excitation_tripped = false;
    _loss_of_excitation_alarmed = false;

    _external_fault_tripped = false;
    _external_fault_alarmed = false;

    _over_flux_warning = false;
    _under_flux_warning = false;
    _stall_warning = false;
    _overspeed_warning = false;
    _machine_overvoltage_warning = false;
    _communication_loss_warning = false;
    _loss_of_excitation_warning = false;
}

void sfc_machine_side_protections::update_vhz_protection(
    voltage_pu_t machine_voltage_magnitude_pu, frequency_pu_t machine_frequency_pu, bool closed_loop_active) noexcept
{
    // VHz protection: Only active during closed-loop operation (observer estimates valid)
    if (closed_loop_active) {
        _vhz_protection.update(machine_voltage_magnitude_pu, machine_frequency_pu);
    }
    else {
        // When not active (e.g. IDLE), drive healthy values (1.0 pu V/Hz).
        // This ensures can_reset() becomes true even if we tripped before shutdown.
        _vhz_protection.update(voltage_pu_t{1.0f}, frequency_pu_t{1.0f});
    }
}

void sfc_machine_side_protections::update_stall_protection(
    current_pu_t dc_link_current_pu, speed_pu_t machine_speed_pu, bool startup_active) noexcept
{
    // Stall protection: Only active during startup (sequential starting)
    if (startup_active) {
        _stall_protection.update(dc_link_current_pu, machine_speed_pu, true);
    }
    else {
        // When not starting, drive healthy values (0 current, 1.0 speed) to allow reset.
        _stall_protection.update(current_pu_t{0.0f}, speed_pu_t{1.0f}, false);
    }
}

void sfc_machine_side_protections::update_overspeed_protection(
    speed_pu_t machine_speed_pu, bool closed_loop_active) noexcept
{
    // Overspeed protection: Only active during closed-loop operation
    if (closed_loop_active) {
        _overspeed_protection.update(machine_speed_pu);
    }
    else {
        // Drive 0.0 pu speed in Standby to allow reset.
        _overspeed_protection.update(speed_pu_t{0.0f});
    }
}

void sfc_machine_side_protections::update_machine_overvoltage_protection(
    voltage_pu_t machine_voltage_magnitude_pu) noexcept
{
    // Machine overvoltage: Always run (rectifier/back-EMF always present)
    _machine_overvoltage_protection.update(machine_voltage_magnitude_pu);
}

void sfc_machine_side_protections::update_loss_of_excitation_protection(
    bool excitation_is_on, bool startup_active, bool closed_loop_active) noexcept
{
    // Loss of excitation protection: Active during startup and closed-loop operation
    if (startup_active || closed_loop_active) {
        _loss_of_excitation_protection.update(excitation_is_on ? real_t{1.0f} : real_t{0.0f});
    }
    else {
        // Drive healthy "ON" value in Standby to allow reset.
        _loss_of_excitation_protection.update(real_t{1.0f});
    }
}

void sfc_machine_side_protections::update_communication_watchdog(bool communication_valid) noexcept
{
    _communication_watchdog.update(communication_valid);
}

void sfc_machine_side_protections::update_external_fault(bool active) noexcept { _external_fault_active = active; }

void sfc_machine_side_protections::resolve_protections(bool module_enabled, bool confirm_trips) noexcept
{
    // If module is disabled, force-clear all internal states and outputs.
    if (!module_enabled) {
        // Force-clear latched protection blocks regardless of current inputs.
        // Some blocks only reset when their measured values are back in a safe range,
        // so drive safe values first, then reset.
        _vhz_protection.update(voltage_pu_t{1.0f}, frequency_pu_t{1.0f});
        _vhz_protection.reset();

        _stall_protection.update(current_pu_t{0.0f}, speed_pu_t{1.0f}, false);
        _stall_protection.reset();

        _overspeed_protection.update(speed_pu_t{0.0f});
        _overspeed_protection.reset();

        _machine_overvoltage_protection.update(voltage_pu_t{0.0f});
        _machine_overvoltage_protection.reset();

        _loss_of_excitation_protection.update(real_t{1.0f});  // Safe value: excitation ON
        _loss_of_excitation_protection.reset();

        _communication_watchdog.reset();
        _trip_handshake.reset();

        _has_active_trip = false;
        _has_active_alarm = false;
        _is_inhibited = false;

        _over_flux_tripped = false;
        _over_flux_alarmed = false;
        _under_flux_tripped = false;
        _under_flux_alarmed = false;

        _stall_tripped = false;
        _stall_alarmed = false;

        _overspeed_tripped = false;
        _overspeed_alarmed = false;

        _machine_overvoltage_tripped = false;
        _machine_overvoltage_alarmed = false;

        _communication_loss_tripped = false;
        _communication_loss_alarmed = false;
        _communication_loss_warning = false;

        _loss_of_excitation_tripped = false;
        _loss_of_excitation_alarmed = false;
        _loss_of_excitation_warning = false;

        _overspeed_tripped = false;
        _overspeed_alarmed = false;
        _overspeed_warning = false;

        _machine_overvoltage_tripped = false;
        _machine_overvoltage_alarmed = false;
        _machine_overvoltage_warning = false;

        _external_fault_tripped = false;
        _external_fault_alarmed = false;

        _over_flux_warning = false;
        _under_flux_warning = false;
        _stall_warning = false;

        return;
    }

    // The reset pulse is triggered when the confirm_trips signal goes true (trip cleared and confirmed).
    _is_confirm_trip_rising_edge_detector.update(confirm_trips);
    const bool reset_trigger = _is_confirm_trip_rising_edge_detector.is_rising_edge();

    // Reset protections that are ready to be reset (condition cleared + no latched trip)
    if (reset_trigger) {
        if (_vhz_protection.can_reset()) {
            _vhz_protection.reset();
        }
        if (_stall_protection.can_reset()) {
            _stall_protection.reset();
        }
        if (_overspeed_protection.can_reset()) {
            _overspeed_protection.reset();
        }
        if (_machine_overvoltage_protection.can_reset()) {
            _machine_overvoltage_protection.reset();
        }
        if (_loss_of_excitation_protection.can_reset()) {
            _loss_of_excitation_protection.reset();
        }

        // Clear the recorded first trip source on user confirmation
        _first_trip_source = machine_trip_source::none;
    }

    const bool over_flux_active = _over_flux_is_trip && _vhz_protection.is_over_flux_active();
    const bool under_flux_active = _under_flux_is_trip && _vhz_protection.is_under_flux_active();
    const bool stall_active = _stall_is_trip && _stall_protection.is_active();
    const bool comm_loss_active = _comm_loss_is_trip && _communication_watchdog.is_active();
    const bool overspeed_active = _overspeed_is_trip && _overspeed_protection.is_active();
    const bool machine_overvoltage_active = _machine_overvoltage_is_trip && _machine_overvoltage_protection.is_active();
    const bool loss_of_excitation_active = _loss_of_excitation_is_trip && _loss_of_excitation_protection.is_active();
    const bool external_fault_active = _external_fault_is_trip && _external_fault_delay.get_output();

    const bool over_flux_trip = _over_flux_is_trip && _vhz_protection.is_over_flux_tripped();
    const bool under_flux_trip = _under_flux_is_trip && _vhz_protection.is_under_flux_tripped();
    const bool stall_trip = _stall_is_trip && _stall_protection.is_tripped();
    const bool comm_loss_trip = _comm_loss_is_trip && _communication_watchdog.is_tripped();
    const bool overspeed_trip = _overspeed_is_trip && _overspeed_protection.is_tripped();
    const bool machine_overvoltage_trip = _machine_overvoltage_is_trip && _machine_overvoltage_protection.is_tripped();
    const bool loss_of_excitation_trip = _loss_of_excitation_is_trip && _loss_of_excitation_protection.is_tripped();
    const bool external_fault_trip =
        _external_fault_is_trip && _external_fault_active;  // external fault doesn't have a latch?

    const bool over_flux_alarm = (!_over_flux_is_trip) && _vhz_protection.is_over_flux_active();
    const bool under_flux_alarm = (!_under_flux_is_trip) && _vhz_protection.is_under_flux_active();
    const bool stall_alarm = (!_stall_is_trip) && _stall_protection.is_active();
    const bool comm_loss_alarm = (!_comm_loss_is_trip) && _communication_watchdog.is_active();
    const bool overspeed_alarm = (!_overspeed_is_trip) && _overspeed_protection.is_active();
    const bool machine_overvoltage_alarm =
        (!_machine_overvoltage_is_trip) && _machine_overvoltage_protection.is_active();
    const bool loss_of_excitation_alarm = (!_loss_of_excitation_is_trip) && _loss_of_excitation_protection.is_active();
    const bool external_fault_alarm = (!_external_fault_is_trip) && _external_fault_active;

    _over_flux_tripped = over_flux_trip;
    _over_flux_alarmed = over_flux_alarm;
    _under_flux_tripped = under_flux_trip;
    _under_flux_alarmed = under_flux_alarm;
    _stall_tripped = stall_trip;
    _stall_alarmed = stall_alarm;
    _communication_loss_tripped = comm_loss_trip;
    _communication_loss_alarmed = comm_loss_alarm;
    _overspeed_tripped = overspeed_trip;
    _overspeed_alarmed = overspeed_alarm;
    _machine_overvoltage_tripped = machine_overvoltage_trip;
    _machine_overvoltage_alarmed = machine_overvoltage_alarm;
    _loss_of_excitation_tripped = loss_of_excitation_trip;
    _loss_of_excitation_alarmed = loss_of_excitation_alarm;
    _external_fault_tripped = external_fault_trip;
    _external_fault_alarmed = external_fault_alarm;

    _over_flux_warning = _vhz_protection.is_over_flux_warning();
    _under_flux_warning = _vhz_protection.is_under_flux_warning();
    _stall_warning = _stall_protection.is_warning();
    _communication_loss_warning = _communication_watchdog.is_warning();
    _overspeed_warning = _overspeed_protection.is_warning();
    _machine_overvoltage_warning = _machine_overvoltage_protection.is_warning();
    _loss_of_excitation_warning = _loss_of_excitation_protection.is_warning();

    const bool any_protection_active = over_flux_active || under_flux_active || stall_active || comm_loss_active ||
                                       overspeed_active || machine_overvoltage_active || loss_of_excitation_active ||
                                       external_fault_active;

    const bool any_protection_tripped = over_flux_trip || under_flux_trip || stall_trip || comm_loss_trip ||
                                        overspeed_trip || machine_overvoltage_trip || loss_of_excitation_trip ||
                                        external_fault_trip;

    // Latch first trip source
    if (_first_trip_source == machine_trip_source::none && any_protection_tripped) {
        if (over_flux_trip) {
            _first_trip_source = machine_trip_source::over_flux;
        }
        else if (under_flux_trip) {
            _first_trip_source = machine_trip_source::under_flux;
        }
        else if (stall_trip) {
            _first_trip_source = machine_trip_source::stall;
        }
        else if (comm_loss_trip) {
            _first_trip_source = machine_trip_source::communication_loss;
        }
        else if (overspeed_trip) {
            _first_trip_source = machine_trip_source::overspeed;
        }
        else if (machine_overvoltage_trip) {
            _first_trip_source = machine_trip_source::machine_overvoltage;
        }
        else if (loss_of_excitation_trip) {
            _first_trip_source = machine_trip_source::loss_of_excitation;
        }
        else if (external_fault_trip) {
            _first_trip_source = machine_trip_source::external_fault;
        }
    }

    const bool any_alarm_active = over_flux_alarm || under_flux_alarm || stall_alarm || comm_loss_alarm ||
                                  overspeed_alarm || machine_overvoltage_alarm || loss_of_excitation_alarm ||
                                  external_fault_alarm;

    // Update the handshake with active fault state (reflects real-time presence of fault)
    _trip_handshake.update(any_protection_active, confirm_trips);

    // has_active_trip reflects real-time state (clears automatically when fault clears)
    _has_active_trip = any_protection_active;
    _has_active_alarm = any_alarm_active;
    _is_inhibited = _trip_handshake.get_is_inhibited();
}

void sfc_machine_side_protections::check_and_reset_handshake_if_no_trips() noexcept
{
    // Check if ANY trip is currently active (after severity routing)
    const bool over_flux_trip = _over_flux_is_trip && _vhz_protection.is_over_flux_tripped();
    const bool under_flux_trip = _under_flux_is_trip && _vhz_protection.is_under_flux_tripped();
    const bool stall_trip = _stall_is_trip && _stall_protection.is_tripped();
    const bool comm_loss_trip = _comm_loss_is_trip && _communication_watchdog.is_tripped();
    const bool overspeed_trip = _overspeed_is_trip && _overspeed_protection.is_tripped();
    const bool machine_overvoltage_trip = _machine_overvoltage_is_trip && _machine_overvoltage_protection.is_tripped();
    const bool loss_of_excitation_trip = _loss_of_excitation_is_trip && _loss_of_excitation_protection.is_tripped();
    const bool external_fault_trip = _external_fault_is_trip && _external_fault_active;

    const bool any_trip_active = over_flux_trip || under_flux_trip || stall_trip || comm_loss_trip || overspeed_trip ||
                                 machine_overvoltage_trip || loss_of_excitation_trip || external_fault_trip;

    // If no trips are active after severity change, reset the handshake to clear inhibit
    if (!any_trip_active) {
        _trip_handshake.reset();
        _is_inhibited = false;
    }
}

void sfc_machine_side_protections::configure_over_flux_is_trip(bool is_trip) noexcept
{
    if (_over_flux_is_trip != is_trip) {
        _vhz_protection.reset();
        _over_flux_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _over_flux_is_trip = is_trip;
}
void sfc_machine_side_protections::configure_under_flux_is_trip(bool is_trip) noexcept
{
    if (_under_flux_is_trip != is_trip) {
        _vhz_protection.reset();
        _under_flux_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _under_flux_is_trip = is_trip;
}
void sfc_machine_side_protections::configure_stall_is_trip(bool is_trip) noexcept
{
    if (_stall_is_trip != is_trip) {
        _stall_protection.reset();
        _stall_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _stall_is_trip = is_trip;
}
void sfc_machine_side_protections::configure_overspeed_is_trip(bool is_trip) noexcept
{
    if (_overspeed_is_trip != is_trip) {
        _overspeed_protection.reset();
        _overspeed_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _overspeed_is_trip = is_trip;
}
void sfc_machine_side_protections::configure_machine_overvoltage_is_trip(bool is_trip) noexcept
{
    if (_machine_overvoltage_is_trip != is_trip) {
        _machine_overvoltage_protection.reset();
        _machine_overvoltage_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _machine_overvoltage_is_trip = is_trip;
}
void sfc_machine_side_protections::configure_communication_loss_is_trip(bool is_trip) noexcept
{
    if (_comm_loss_is_trip != is_trip) {
        _communication_watchdog.reset();
        _comm_loss_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _comm_loss_is_trip = is_trip;
}
void sfc_machine_side_protections::configure_loss_of_excitation_is_trip(bool is_trip) noexcept
{
    if (_loss_of_excitation_is_trip != is_trip) {
        _loss_of_excitation_protection.reset();
        _loss_of_excitation_is_trip = is_trip;
        check_and_reset_handshake_if_no_trips();
        return;
    }
    _loss_of_excitation_is_trip = is_trip;
}

void sfc_machine_side_protections::configure_over_flux_trip_threshold(flux_pu_t threshold) noexcept
{
    _over_flux_trip_threshold = threshold;
    _vhz_protection.configure_over_flux_trip_threshold(threshold);
}
void sfc_machine_side_protections::configure_over_flux_warning_threshold(flux_pu_t threshold) noexcept
{
    _vhz_protection.configure_over_flux_warning_threshold(threshold);
}
void sfc_machine_side_protections::configure_over_flux_trip_delay(duration_t delay) noexcept
{
    _vhz_protection.configure_over_flux_trip_delay(delay);
}
void sfc_machine_side_protections::configure_over_flux_warning_delay(duration_t delay) noexcept
{
    _vhz_protection.configure_over_flux_warning_delay(delay);
}

void sfc_machine_side_protections::configure_under_flux_trip_threshold(flux_pu_t threshold) noexcept
{
    _under_flux_trip_threshold = threshold;
    _vhz_protection.configure_under_flux_trip_threshold(threshold);
}
void sfc_machine_side_protections::configure_under_flux_warning_threshold(flux_pu_t threshold) noexcept
{
    _vhz_protection.configure_under_flux_warning_threshold(threshold);
}
void sfc_machine_side_protections::configure_under_flux_trip_delay(duration_t delay) noexcept
{
    _vhz_protection.configure_under_flux_trip_delay(delay);
}
void sfc_machine_side_protections::configure_under_flux_warning_delay(duration_t delay) noexcept
{
    _vhz_protection.configure_under_flux_warning_delay(delay);
}

void sfc_machine_side_protections::configure_stall_current_threshold(current_pu_t threshold) noexcept
{
    _stall_protection.configure_current_threshold(threshold);
}
void sfc_machine_side_protections::configure_stall_speed_threshold(speed_pu_t threshold) noexcept
{
    _stall_protection.configure_speed_threshold(threshold);
}
void sfc_machine_side_protections::configure_stall_trip_delay(duration_t delay) noexcept
{
    _stall_protection.configure_trip_delay(delay);
}
void sfc_machine_side_protections::configure_stall_warning_delay(duration_t delay) noexcept
{
    _stall_protection.configure_warning_delay(delay);
}

void sfc_machine_side_protections::configure_comm_timeout_duration(duration_t timeout) noexcept
{
    _communication_watchdog.configure_timeout_duration(timeout);
}
void sfc_machine_side_protections::configure_comm_reset_duration(duration_t reset) noexcept
{
    _communication_watchdog.configure_reset_duration(reset);
}

void sfc_machine_side_protections::configure_overspeed_trip_threshold(speed_pu_t threshold) noexcept
{
    _overspeed_trip_threshold = threshold;
    _overspeed_protection.configure_trip_threshold(threshold);
}
void sfc_machine_side_protections::configure_overspeed_warning_threshold(speed_pu_t threshold) noexcept
{
    _overspeed_protection.configure_warning_threshold(threshold);
}
void sfc_machine_side_protections::configure_overspeed_trip_delay(duration_t delay) noexcept
{
    _overspeed_protection.configure_trip_delay(delay);
}
void sfc_machine_side_protections::configure_overspeed_warning_delay(duration_t delay) noexcept
{
    _overspeed_protection.configure_warning_delay(delay);
}

void sfc_machine_side_protections::configure_machine_overvoltage_trip_threshold(voltage_pu_t threshold) noexcept
{
    _machine_overvoltage_trip_threshold = threshold;
    _machine_overvoltage_protection.configure_trip_threshold(threshold);
}
void sfc_machine_side_protections::configure_machine_overvoltage_warning_threshold(voltage_pu_t threshold) noexcept
{
    _machine_overvoltage_protection.configure_warning_threshold(threshold);
}
void sfc_machine_side_protections::configure_machine_overvoltage_trip_delay(duration_t delay) noexcept
{
    _machine_overvoltage_protection.configure_trip_delay(delay);
}
void sfc_machine_side_protections::configure_machine_overvoltage_warning_delay(duration_t delay) noexcept
{
    _machine_overvoltage_protection.configure_warning_delay(delay);
}

void sfc_machine_side_protections::configure_loss_of_excitation_trip_threshold(real_t threshold) noexcept
{
    _loss_of_excitation_protection.configure_trip_threshold(threshold);
}
void sfc_machine_side_protections::configure_loss_of_excitation_warning_threshold(real_t threshold) noexcept
{
    _loss_of_excitation_protection.configure_warning_threshold(threshold);
}
void sfc_machine_side_protections::configure_loss_of_excitation_trip_delay(duration_t delay) noexcept
{
    _loss_of_excitation_protection.configure_trip_delay(delay);
}
void sfc_machine_side_protections::configure_loss_of_excitation_warning_delay(duration_t delay) noexcept
{
    _loss_of_excitation_protection.configure_warning_delay(delay);
}

void sfc_machine_side_protections::configure_external_fault_trip_delay(duration_t delay) noexcept
{
    _external_fault_delay.configure(delay.value());
}

bool sfc_machine_side_protections::get_has_active_trip() const noexcept { return _has_active_trip; }
bool sfc_machine_side_protections::get_has_active_alarm() const noexcept { return _has_active_alarm; }
bool sfc_machine_side_protections::get_is_inhibited() const noexcept { return _is_inhibited; }

bool sfc_machine_side_protections::get_over_flux_tripped() const noexcept { return _over_flux_tripped; }
bool sfc_machine_side_protections::get_over_flux_alarmed() const noexcept { return _over_flux_alarmed; }
bool sfc_machine_side_protections::get_under_flux_tripped() const noexcept { return _under_flux_tripped; }
bool sfc_machine_side_protections::get_under_flux_alarmed() const noexcept { return _under_flux_alarmed; }

bool sfc_machine_side_protections::get_stall_tripped() const noexcept { return _stall_tripped; }
bool sfc_machine_side_protections::get_stall_alarmed() const noexcept { return _stall_alarmed; }

bool sfc_machine_side_protections::get_overspeed_tripped() const noexcept { return _overspeed_tripped; }
bool sfc_machine_side_protections::get_overspeed_alarmed() const noexcept { return _overspeed_alarmed; }
bool sfc_machine_side_protections::get_overspeed_warning() const noexcept { return _overspeed_warning; }

bool sfc_machine_side_protections::get_machine_overvoltage_tripped() const noexcept
{
    return _machine_overvoltage_tripped;
}
bool sfc_machine_side_protections::get_machine_overvoltage_alarmed() const noexcept
{
    return _machine_overvoltage_alarmed;
}
bool sfc_machine_side_protections::get_machine_overvoltage_warning() const noexcept
{
    return _machine_overvoltage_warning;
}

bool sfc_machine_side_protections::get_communication_loss_tripped() const noexcept
{
    return _communication_loss_tripped;
}
bool sfc_machine_side_protections::get_communication_loss_alarmed() const noexcept
{
    return _communication_loss_alarmed;
}

bool sfc_machine_side_protections::get_loss_of_excitation_tripped() const noexcept
{
    return _loss_of_excitation_tripped;
}
bool sfc_machine_side_protections::get_loss_of_excitation_alarmed() const noexcept
{
    return _loss_of_excitation_alarmed;
}
bool sfc_machine_side_protections::get_loss_of_excitation_warning() const noexcept
{
    return _loss_of_excitation_warning;
}

bool sfc_machine_side_protections::get_external_fault_tripped() const noexcept { return _external_fault_tripped; }
bool sfc_machine_side_protections::get_external_fault_alarmed() const noexcept { return _external_fault_alarmed; }

void sfc_machine_side_protections::configure_external_fault_is_trip(bool is_trip) noexcept
{
    _external_fault_is_trip = is_trip;
}

bool sfc_machine_side_protections::get_over_flux_warning() const noexcept { return _over_flux_warning; }
bool sfc_machine_side_protections::get_under_flux_warning() const noexcept { return _under_flux_warning; }
bool sfc_machine_side_protections::get_stall_warning() const noexcept { return _stall_warning; }
bool sfc_machine_side_protections::get_communication_loss_warning() const noexcept
{
    return _communication_loss_warning;
}

flux_pu_t sfc_machine_side_protections::get_vhz_ratio_pu() const noexcept { return _vhz_protection.get_vhz_ratio_pu(); }

}  // namespace yeet::application::sfc
