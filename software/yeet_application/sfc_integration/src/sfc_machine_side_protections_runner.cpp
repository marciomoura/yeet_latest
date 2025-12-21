#include "sfc_integration/sfc_machine_side_protections_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_machine_side_protections_runner::sfc_machine_side_protections_runner()
    : base_application(sampling_time), _protections(sampling_time)
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
    _protections.reset();

    // Clear communication status
    _last_communication_valid = true;

    // Clear all output signals
    _outputs.is_tripped = false;
    _outputs.is_inhibited = false;
    _outputs.any_fault_active = false;
    _outputs.over_flux_tripped = false;
    _outputs.over_flux_alarmed = false;
    _outputs.under_flux_tripped = false;
    _outputs.under_flux_alarmed = false;
    _outputs.stall_tripped = false;
    _outputs.stall_alarmed = false;
    _outputs.over_flux_warning = false;
    _outputs.under_flux_warning = false;
    _outputs.stall_warning = false;
    _outputs.overspeed_tripped = false;
    _outputs.overspeed_alarmed = false;
    _outputs.machine_overvoltage_tripped = false;
    _outputs.machine_overvoltage_alarmed = false;
    _outputs.communication_loss_tripped = false;
    _outputs.communication_loss_alarmed = false;
    _outputs.communication_loss_warning = false;
    _outputs.loss_of_excitation_tripped = false;
    _outputs.loss_of_excitation_alarmed = false;
}

void sfc_machine_side_protections_runner::run()
{
    // Handle manual reset request (for testing/commissioning)
    if (_parameters.force_reset_all) {
        reset();
        _parameters.force_reset_all = false;  // Clear flag after reset
        return;                               // Skip normal protection logic this cycle to allow clean reset
    }

    // Only update protections when both module is enabled AND release signal is active
    const bool module_enabled = _parameters.protection_module_enabled;
    const bool release_signal = _inputs.release_protections;

    if (module_enabled) {
        _protections.update_communication_watchdog(_last_communication_valid);
        _protections.update_vhz_protection(_inputs.machine_voltage_magnitude_pu.read(),
            _inputs.machine_frequency_pu.read(), _inputs.closed_loop_active.read());
        _protections.update_stall_protection(
            _inputs.dc_link_current_pu.read(), _inputs.machine_speed.read(), _inputs.startup_active.read());
        _protections.update_overspeed_protection(_inputs.machine_speed.read(), _inputs.closed_loop_active.read());
        _protections.update_machine_overvoltage_protection(_inputs.machine_voltage_magnitude_pu.read());
        _protections.update_loss_of_excitation_protection(
            _inputs.excitation_is_on.read(), _inputs.startup_active.read(), _inputs.closed_loop_active.read());
    }

    _protections.resolve_protections(module_enabled, _parameters.confirm_alarms.read());

    _outputs.is_tripped = _protections.get_is_tripped();
    _outputs.is_inhibited = _protections.get_is_inhibited();
    _outputs.any_fault_active = _protections.get_any_fault_active();

    _outputs.over_flux_tripped = _protections.get_over_flux_tripped();
    _outputs.over_flux_alarmed = _protections.get_over_flux_alarmed();
    _outputs.under_flux_tripped = _protections.get_under_flux_tripped();
    _outputs.under_flux_alarmed = _protections.get_under_flux_alarmed();
    _outputs.stall_tripped = _protections.get_stall_tripped();
    _outputs.stall_alarmed = _protections.get_stall_alarmed();
    _outputs.over_flux_warning = _protections.get_over_flux_warning();
    _outputs.under_flux_warning = _protections.get_under_flux_warning();
    _outputs.stall_warning = _protections.get_stall_warning();

    _outputs.overspeed_tripped = _protections.get_overspeed_tripped();
    _outputs.overspeed_alarmed = _protections.get_overspeed_alarmed();
    _outputs.machine_overvoltage_tripped = _protections.get_machine_overvoltage_tripped();
    _outputs.machine_overvoltage_alarmed = _protections.get_machine_overvoltage_alarmed();

    _outputs.communication_loss_tripped = _protections.get_communication_loss_tripped();
    _outputs.communication_loss_alarmed = _protections.get_communication_loss_alarmed();
    _outputs.communication_loss_warning = _protections.get_communication_loss_warning();

    _outputs.loss_of_excitation_tripped = _protections.get_loss_of_excitation_tripped();
    _outputs.loss_of_excitation_alarmed = _protections.get_loss_of_excitation_alarmed();
}

void sfc_machine_side_protections_runner::stop() { reset(); }

void sfc_machine_side_protections_runner::update_communication_status(bool communication_valid)
{
    _last_communication_valid = communication_valid;
}

void sfc_machine_side_protections_runner::set_parameters()
{
    // V/Hz protection configuration
    _protections.configure_over_flux_trip_threshold(_parameters.over_flux_trip_threshold.read());
    _protections.configure_over_flux_warning_threshold(_parameters.over_flux_warning_threshold.read());
    _protections.configure_over_flux_trip_delay(_parameters.over_flux_trip_delay.read());
    _protections.configure_over_flux_warning_delay(_parameters.over_flux_warning_delay.read());
    _protections.configure_under_flux_trip_threshold(_parameters.under_flux_trip_threshold.read());
    _protections.configure_under_flux_warning_threshold(_parameters.under_flux_warning_threshold.read());
    _protections.configure_under_flux_trip_delay(_parameters.under_flux_trip_delay.read());
    _protections.configure_under_flux_warning_delay(_parameters.under_flux_warning_delay.read());

    // Stall protection configuration
    _protections.configure_stall_current_threshold(_parameters.stall_current_threshold.read());
    _protections.configure_stall_speed_threshold(_parameters.stall_speed_threshold.read());
    _protections.configure_stall_trip_delay(_parameters.stall_trip_delay.read());
    _protections.configure_stall_warning_delay(_parameters.stall_warning_delay.read());

    // Communication watchdog configuration
    _protections.configure_comm_timeout_duration(_parameters.comm_timeout.read());
    _protections.configure_comm_reset_duration(_parameters.comm_reset.read());

    // Overspeed protection configuration
    _protections.configure_overspeed_trip_threshold(_parameters.overspeed_trip_threshold.read());
    _protections.configure_overspeed_trip_delay(_parameters.overspeed_trip_delay.read());

    // Machine-side overvoltage protection configuration
    _protections.configure_machine_overvoltage_trip_threshold(_parameters.machine_overvoltage_trip_threshold.read());
    _protections.configure_machine_overvoltage_trip_delay(_parameters.machine_overvoltage_trip_delay.read());

    // Loss of excitation protection configuration
    _protections.configure_loss_of_excitation_trip_delay(_parameters.loss_of_excitation_trip_delay.read());

    // Severity routing configuration
    _protections.configure_over_flux_is_trip(_parameters.over_flux_severity == protection_severity::trip);
    _protections.configure_under_flux_is_trip(_parameters.under_flux_severity == protection_severity::trip);
    _protections.configure_stall_is_trip(_parameters.stall_severity == protection_severity::trip);
    _protections.configure_overspeed_is_trip(_parameters.overspeed_severity == protection_severity::trip);
    _protections.configure_machine_overvoltage_is_trip(
        _parameters.machine_overvoltage_severity == protection_severity::trip);
    _protections.configure_communication_loss_is_trip(
        _parameters.communication_loss_severity == protection_severity::trip);
    _protections.configure_loss_of_excitation_is_trip(
        _parameters.loss_of_excitation_severity == protection_severity::trip);
}

void sfc_machine_side_protections_runner::set_parameter_callback()
{
    // Register callbacks for V/Hz protection parameters
    _parameters.over_flux_trip_threshold.register_callback(
        [&]() { _protections.configure_over_flux_trip_threshold(_parameters.over_flux_trip_threshold); });
    _parameters.over_flux_warning_threshold.register_callback(
        [&]() { _protections.configure_over_flux_warning_threshold(_parameters.over_flux_warning_threshold); });
    _parameters.over_flux_trip_delay.register_callback(
        [&]() { _protections.configure_over_flux_trip_delay(_parameters.over_flux_trip_delay); });
    _parameters.over_flux_warning_delay.register_callback(
        [&]() { _protections.configure_over_flux_warning_delay(_parameters.over_flux_warning_delay); });
    _parameters.under_flux_trip_threshold.register_callback(
        [&]() { _protections.configure_under_flux_trip_threshold(_parameters.under_flux_trip_threshold); });
    _parameters.under_flux_warning_threshold.register_callback(
        [&]() { _protections.configure_under_flux_warning_threshold(_parameters.under_flux_warning_threshold); });
    _parameters.under_flux_trip_delay.register_callback(
        [&]() { _protections.configure_under_flux_trip_delay(_parameters.under_flux_trip_delay); });
    _parameters.under_flux_warning_delay.register_callback(
        [&]() { _protections.configure_under_flux_warning_delay(_parameters.under_flux_warning_delay); });

    // Register callbacks for stall protection parameters
    _parameters.stall_current_threshold.register_callback(
        [&]() { _protections.configure_stall_current_threshold(_parameters.stall_current_threshold); });
    _parameters.stall_speed_threshold.register_callback(
        [&]() { _protections.configure_stall_speed_threshold(_parameters.stall_speed_threshold); });
    _parameters.stall_trip_delay.register_callback(
        [&]() { _protections.configure_stall_trip_delay(_parameters.stall_trip_delay); });
    _parameters.stall_warning_delay.register_callback(
        [&]() { _protections.configure_stall_warning_delay(_parameters.stall_warning_delay); });

    // Register callbacks for communication watchdog parameters
    _parameters.comm_timeout.register_callback(
        [&]() { _protections.configure_comm_timeout_duration(_parameters.comm_timeout); });
    _parameters.comm_reset.register_callback(
        [&]() { _protections.configure_comm_reset_duration(_parameters.comm_reset); });

    // Register callbacks for overspeed and machine-side overvoltage parameters
    _parameters.overspeed_trip_threshold.register_callback(
        [&]() { _protections.configure_overspeed_trip_threshold(_parameters.overspeed_trip_threshold); });
    _parameters.overspeed_trip_delay.register_callback(
        [&]() { _protections.configure_overspeed_trip_delay(_parameters.overspeed_trip_delay); });
    _parameters.machine_overvoltage_trip_threshold.register_callback([&]() {
        _protections.configure_machine_overvoltage_trip_threshold(_parameters.machine_overvoltage_trip_threshold);
    });
    _parameters.machine_overvoltage_trip_delay.register_callback(
        [&]() { _protections.configure_machine_overvoltage_trip_delay(_parameters.machine_overvoltage_trip_delay); });

    // Register callbacks for loss of excitation parameters
    _parameters.loss_of_excitation_trip_delay.register_callback(
        [&]() { _protections.configure_loss_of_excitation_trip_delay(_parameters.loss_of_excitation_trip_delay); });

    // Register callbacks for severity parameters
    _parameters.over_flux_severity.register_callback([&]() {
        _protections.configure_over_flux_is_trip(_parameters.over_flux_severity == protection_severity::trip);
    });
    _parameters.under_flux_severity.register_callback([&]() {
        _protections.configure_under_flux_is_trip(_parameters.under_flux_severity == protection_severity::trip);
    });
    _parameters.stall_severity.register_callback(
        [&]() { _protections.configure_stall_is_trip(_parameters.stall_severity == protection_severity::trip); });
    _parameters.overspeed_severity.register_callback([&]() {
        _protections.configure_overspeed_is_trip(_parameters.overspeed_severity == protection_severity::trip);
    });
    _parameters.machine_overvoltage_severity.register_callback([&]() {
        _protections.configure_machine_overvoltage_is_trip(
            _parameters.machine_overvoltage_severity == protection_severity::trip);
    });
    _parameters.communication_loss_severity.register_callback([&]() {
        _protections.configure_communication_loss_is_trip(
            _parameters.communication_loss_severity == protection_severity::trip);
    });
    _parameters.loss_of_excitation_severity.register_callback([&]() {
        _protections.configure_loss_of_excitation_is_trip(
            _parameters.loss_of_excitation_severity == protection_severity::trip);
    });
}

}  // namespace yeet::application::sfc::integration
