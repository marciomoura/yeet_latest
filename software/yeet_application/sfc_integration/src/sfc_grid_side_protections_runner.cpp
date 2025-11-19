#include "sfc_integration/sfc_grid_side_protections_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_protections_runner::sfc_grid_side_protections_runner()
    : base_application(sampling_time),
      _overcurrent_protection(sampling_time),
      _overvoltage_protection(sampling_time),
      _undervoltage_protection(sampling_time),
      _communication_watchdog(sampling_time)
{
}

void sfc_grid_side_protections_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_protections_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_grid_side_protections_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    reset();
}

void sfc_grid_side_protections_runner::reset()
{
    _overcurrent_protection.reset();
    _overvoltage_protection.reset();
    _undervoltage_protection.reset();
    _communication_watchdog.reset();

    // Clear trip latch and alarm pending state
    _trip_latched = false;
    _alarm_pending = false;

    // Clear all output signals
    _outputs.is_tripped = false;
    _outputs.is_inhibited = false;
    _outputs.any_fault_active = false;
    _outputs.overcurrent_tripped = false;
    _outputs.overvoltage_tripped = false;
    _outputs.undervoltage_tripped = false;
    _outputs.overcurrent_warning = false;
    _outputs.overvoltage_warning = false;
    _outputs.undervoltage_warning = false;
    _outputs.communication_loss_tripped = false;
    _outputs.communication_loss_warning = false;
}

void sfc_grid_side_protections_runner::run()
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
        _overcurrent_protection.reset();
        _overvoltage_protection.reset();
        _undervoltage_protection.reset();
        _communication_watchdog.reset();
        _trip_latched = false;
        _alarm_pending = false;

        // Clear all output signals immediately
        _outputs.is_tripped = false;
        _outputs.is_inhibited = false;
        _outputs.any_fault_active = false;
        _outputs.overcurrent_tripped = false;
        _outputs.overvoltage_tripped = false;
        _outputs.undervoltage_tripped = false;
        _outputs.overcurrent_warning = false;
        _outputs.overvoltage_warning = false;
        _outputs.undervoltage_warning = false;
        _outputs.communication_loss_tripped = false;
        _outputs.communication_loss_warning = false;
        return;  // Skip normal protection logic
    }
    else if (enable) {
        // Update all protections with latest measurements
        _overcurrent_protection.update(_inputs.dc_link_current_pu);
        _overvoltage_protection.update(_inputs.grid_voltage_magnitude_pu);
        _undervoltage_protection.update(_inputs.grid_voltage_magnitude_pu);

        // Attempt to reset protections when release signal is active and fault condition has cleared
        // This allows latched protections to release when the measured value returns to safe range
        if (_overcurrent_protection.can_reset()) {
            _overcurrent_protection.reset();
        }
        if (_overvoltage_protection.can_reset()) {
            _overvoltage_protection.reset();
        }
        if (_undervoltage_protection.can_reset()) {
            _undervoltage_protection.reset();
        }
    }

    // Read individual protection states
    bool overcurrent_trip = _overcurrent_protection.is_tripped();
    bool overvoltage_trip = _overvoltage_protection.is_tripped();
    bool undervoltage_trip = _undervoltage_protection.is_tripped();

    // Update individual protection outputs
    _outputs.overcurrent_tripped = overcurrent_trip;
    _outputs.overvoltage_tripped = overvoltage_trip;
    _outputs.undervoltage_tripped = undervoltage_trip;

    _outputs.overcurrent_warning = _overcurrent_protection.is_warning();
    _outputs.overvoltage_warning = _overvoltage_protection.is_warning();
    _outputs.undervoltage_warning = _undervoltage_protection.is_warning();

    // ==================== Trip Handshake Logic ====================
    // Combined trip from individual protections (NOT latched - clears immediately)
    bool any_protection_tripped = overcurrent_trip || overvoltage_trip || undervoltage_trip;

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

void sfc_grid_side_protections_runner::stop() { reset(); }

void sfc_grid_side_protections_runner::set_parameters()
{
    // Overcurrent protection configuration
    _overcurrent_protection.configure_trip_threshold(_parameters.overcurrent_trip_threshold);
    _overcurrent_protection.configure_warning_threshold(_parameters.overcurrent_warning_threshold);
    _overcurrent_protection.configure_trip_delay(_parameters.overcurrent_trip_delay);
    _overcurrent_protection.configure_warning_delay(_parameters.overcurrent_warning_delay);

    // Overvoltage protection configuration
    _overvoltage_protection.configure_trip_threshold(_parameters.overvoltage_trip_threshold);
    _overvoltage_protection.configure_warning_threshold(_parameters.overvoltage_warning_threshold);
    _overvoltage_protection.configure_trip_delay(_parameters.overvoltage_trip_delay);
    _overvoltage_protection.configure_warning_delay(_parameters.overvoltage_warning_delay);

    // Undervoltage protection configuration
    _undervoltage_protection.configure_trip_threshold(_parameters.undervoltage_trip_threshold);
    _undervoltage_protection.configure_warning_threshold(_parameters.undervoltage_warning_threshold);
    _undervoltage_protection.configure_trip_delay(_parameters.undervoltage_trip_delay);
    _undervoltage_protection.configure_warning_delay(_parameters.undervoltage_warning_delay);

    // Communication watchdog configuration
    _communication_watchdog.configure_timeout_duration(_parameters.comm_timeout);
    _communication_watchdog.configure_reset_duration(_parameters.comm_reset);
}

void sfc_grid_side_protections_runner::set_parameter_callback()
{
    // Register callbacks for all protection parameters
    _parameters.overcurrent_trip_threshold.register_callback([&]() { set_parameters(); });
    _parameters.overcurrent_warning_threshold.register_callback([&]() { set_parameters(); });
    _parameters.overcurrent_trip_delay.register_callback([&]() { set_parameters(); });
    _parameters.overcurrent_warning_delay.register_callback([&]() { set_parameters(); });

    _parameters.overvoltage_trip_threshold.register_callback([&]() { set_parameters(); });
    _parameters.overvoltage_warning_threshold.register_callback([&]() { set_parameters(); });
    _parameters.overvoltage_trip_delay.register_callback([&]() { set_parameters(); });
    _parameters.overvoltage_warning_delay.register_callback([&]() { set_parameters(); });

    _parameters.undervoltage_trip_threshold.register_callback([&]() { set_parameters(); });
    _parameters.undervoltage_warning_threshold.register_callback([&]() { set_parameters(); });
    _parameters.undervoltage_trip_delay.register_callback([&]() { set_parameters(); });
    _parameters.undervoltage_warning_delay.register_callback([&]() { set_parameters(); });

    // Register callbacks for communication watchdog parameters
    _parameters.comm_timeout.register_callback([&]() { set_parameters(); });
    _parameters.comm_reset.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
