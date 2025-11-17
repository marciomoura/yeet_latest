#include "sfc_integration/sfc_machine_side_protections_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_machine_side_protections_runner::sfc_machine_side_protections_runner()
    : base_application(sampling_time),
      _vhz_protection(sampling_time),
      _stall_protection(sampling_time),
      _communication_watchdog(sampling_time)
{
}

void sfc_machine_side_protections_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_protections_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_machine_side_protections_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    reset();
}

void sfc_machine_side_protections_runner::reset()
{
    _vhz_protection.reset();
    _stall_protection.reset();
    _communication_watchdog.reset();

    // Clear communication status
    _last_communication_valid = true;

    // Clear trip latch and alarm pending state
    _trip_latched = false;
    _alarm_pending = false;

    // Clear all output signals
    _outputs.is_tripped = false;
    _outputs.is_inhibited = false;
    _outputs.any_fault_active = false;
    _outputs.over_flux_tripped = false;
    _outputs.under_flux_tripped = false;
    _outputs.stall_tripped = false;
    _outputs.over_flux_warning = false;
    _outputs.under_flux_warning = false;
    _outputs.stall_warning = false;
    _outputs.communication_loss_tripped = false;
    _outputs.communication_loss_warning = false;
}

void sfc_machine_side_protections_runner::run()
{
    // Handle manual reset request (for testing/commissioning)
    if (_parameters.force_reset_all) {
        reset();
        _parameters.force_reset_all = false;  // Clear flag after reset
    }

    // Only update protections when both module is enabled AND release signal is active
    bool module_enabled = _parameters.protection_module_enabled;
    bool release_signal = _inputs.release_protections;
    bool enable = module_enabled && release_signal;

    // If module is disabled, force clear all trip states and alarms
    if (!module_enabled) {
        _vhz_protection.reset();
        _stall_protection.reset();
        _communication_watchdog.reset();
        _trip_latched = false;
        _alarm_pending = false;

        // Clear all output signals immediately
        _outputs.is_tripped = false;
        _outputs.is_inhibited = false;
        _outputs.any_fault_active = false;
        _outputs.over_flux_tripped = false;
        _outputs.under_flux_tripped = false;
        _outputs.stall_tripped = false;
        _outputs.over_flux_warning = false;
        _outputs.under_flux_warning = false;
        _outputs.stall_warning = false;
        _outputs.communication_loss_tripped = false;
        _outputs.communication_loss_warning = false;
        return;  // Skip normal protection logic
    }
    else if (enable) {
        // Update V/Hz protection with latest measurements
        _vhz_protection.update(_inputs.machine_voltage_magnitude_pu, _inputs.machine_frequency);

        // Update stall protection with latest measurements and startup state
        _stall_protection.update(_inputs.dc_link_current_pu, _inputs.machine_speed, _inputs.startup_active);

        // Update communication watchdog (uses communication status set via update_communication_status())
        _communication_watchdog.update(_last_communication_valid);

        // Attempt to reset protections when release signal is active and fault condition has cleared
        // This allows latched protections to release when the measured value returns to safe range
        if (_vhz_protection.can_reset()) {
            _vhz_protection.reset();
        }
        if (_stall_protection.can_reset()) {
            _stall_protection.reset();
        }
    }

    // Read individual protection states
    bool over_flux_trip = _vhz_protection.is_over_flux_tripped();
    bool under_flux_trip = _vhz_protection.is_under_flux_tripped();
    bool stall_trip = _stall_protection.is_tripped();
    bool comm_loss_trip = _communication_watchdog.is_tripped();

    // Update individual protection outputs
    _outputs.over_flux_tripped = over_flux_trip;
    _outputs.under_flux_tripped = under_flux_trip;
    _outputs.stall_tripped = stall_trip;
    _outputs.communication_loss_tripped = comm_loss_trip;

    _outputs.over_flux_warning = _vhz_protection.is_over_flux_warning();
    _outputs.under_flux_warning = _vhz_protection.is_under_flux_warning();
    _outputs.stall_warning = _stall_protection.is_warning();
    _outputs.communication_loss_warning = _communication_watchdog.is_warning();

    // ==================== Trip Handshake Logic ====================
    // Combined trip from individual protections (NOT latched - clears immediately)
    bool any_protection_tripped = over_flux_trip || under_flux_trip || stall_trip || comm_loss_trip;

    // Output immediate fault status (for state machine to know when fault condition clears)
    _outputs.any_fault_active = any_protection_tripped;

    // Set trip latch when any protection trips
    if (any_protection_tripped) {
        _trip_latched = true;
    }

    // Clear trip latch logic:
    // - If alarm is pending (inhibited), trip can ONLY be cleared by state machine acknowledgment
    // - If alarm is not pending, trip auto-clears when fault condition clears (transient fault)
    bool can_clear_trip = false;
    if (_alarm_pending) {
        // Shutdown mode: Requires state machine acknowledgment
        can_clear_trip = _inputs.trip_acknowledged;
    }
    else {
        // Normal operation mode: Auto-clear when fault clears
        can_clear_trip = !any_protection_tripped;
    }

    if (can_clear_trip) {
        _trip_latched = false;
    }

    // Output latched trip signal
    _outputs.is_tripped = _trip_latched;

    // ==================== Alarm Confirmation Logic ====================
    // Set alarm pending when trip latch is set
    if (_trip_latched) {
        _alarm_pending = true;
    }

    // Detect rising edge on confirm_alarms parameter
    bool confirm_edge = _confirm_alarms_edge.update(_parameters.confirm_alarms);

    // Clear alarm pending on confirm_alarms rising edge, but ONLY if trip has been acknowledged
    // Safety: Cannot confirm alarms while trip is still latched
    if (confirm_edge && !_trip_latched) {
        _alarm_pending = false;
    }

    // Output is_inhibited status (latching alarm requiring confirmation)
    _outputs.is_inhibited = _alarm_pending;
}

void sfc_machine_side_protections_runner::stop() { reset(); }

void sfc_machine_side_protections_runner::update_communication_status(bool communication_valid)
{
    _last_communication_valid = communication_valid;
}

void sfc_machine_side_protections_runner::set_parameters()
{
    // V/Hz protection configuration
    _vhz_protection.configure_rated_vhz(_parameters.rated_vhz);
    _vhz_protection.configure_over_flux_trip_threshold(_parameters.over_flux_trip_threshold);
    _vhz_protection.configure_over_flux_warning_threshold(_parameters.over_flux_warning_threshold);
    _vhz_protection.configure_over_flux_trip_delay(_parameters.over_flux_trip_delay);
    _vhz_protection.configure_over_flux_warning_delay(_parameters.over_flux_warning_delay);
    _vhz_protection.configure_under_flux_trip_threshold(_parameters.under_flux_trip_threshold);
    _vhz_protection.configure_under_flux_warning_threshold(_parameters.under_flux_warning_threshold);
    _vhz_protection.configure_under_flux_trip_delay(_parameters.under_flux_trip_delay);
    _vhz_protection.configure_under_flux_warning_delay(_parameters.under_flux_warning_delay);

    // Stall protection configuration
    _stall_protection.configure_current_threshold(_parameters.stall_current_threshold);
    _stall_protection.configure_speed_threshold(_parameters.stall_speed_threshold);
    _stall_protection.configure_trip_delay(_parameters.stall_trip_delay);
    _stall_protection.configure_warning_delay(_parameters.stall_warning_delay);

    // Communication watchdog configuration
    _communication_watchdog.configure_timeout_duration(_parameters.comm_timeout);
    _communication_watchdog.configure_reset_duration(_parameters.comm_reset);
}

void sfc_machine_side_protections_runner::set_parameter_callback()
{
    // Register callbacks for V/Hz protection parameters
    _parameters.rated_vhz.register_callback([&]() { set_parameters(); });
    _parameters.over_flux_trip_threshold.register_callback([&]() { set_parameters(); });
    _parameters.over_flux_warning_threshold.register_callback([&]() { set_parameters(); });
    _parameters.over_flux_trip_delay.register_callback([&]() { set_parameters(); });
    _parameters.over_flux_warning_delay.register_callback([&]() { set_parameters(); });
    _parameters.under_flux_trip_threshold.register_callback([&]() { set_parameters(); });
    _parameters.under_flux_warning_threshold.register_callback([&]() { set_parameters(); });
    _parameters.under_flux_trip_delay.register_callback([&]() { set_parameters(); });
    _parameters.under_flux_warning_delay.register_callback([&]() { set_parameters(); });

    // Register callbacks for stall protection parameters
    _parameters.stall_current_threshold.register_callback([&]() { set_parameters(); });
    _parameters.stall_speed_threshold.register_callback([&]() { set_parameters(); });
    _parameters.stall_trip_delay.register_callback([&]() { set_parameters(); });
    _parameters.stall_warning_delay.register_callback([&]() { set_parameters(); });

    // Register callbacks for communication watchdog parameters
    _parameters.comm_timeout.register_callback([&]() { set_parameters(); });
    _parameters.comm_reset.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
