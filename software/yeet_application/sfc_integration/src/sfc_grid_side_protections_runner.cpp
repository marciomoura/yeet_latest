#include "sfc_integration/sfc_grid_side_protections_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_protections_runner::sfc_grid_side_protections_runner()
    : base_application(sampling_time), _protections(sampling_time)
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
    _protections.reset();

    _outputs.is_tripped = _protections.get_is_tripped();
    _outputs.is_inhibited = _protections.get_is_inhibited();
    _outputs.any_fault_active = _protections.get_any_fault_active();

    _outputs.overcurrent_tripped = _protections.get_overcurrent_tripped();
    _outputs.overcurrent_alarmed = _protections.get_overcurrent_alarmed();
    _outputs.overvoltage_tripped = _protections.get_overvoltage_tripped();
    _outputs.overvoltage_alarmed = _protections.get_overvoltage_alarmed();
    _outputs.undervoltage_tripped = _protections.get_undervoltage_tripped();
    _outputs.undervoltage_alarmed = _protections.get_undervoltage_alarmed();
    _outputs.overcurrent_warning = _protections.get_overcurrent_warning();
    _outputs.overvoltage_warning = _protections.get_overvoltage_warning();
    _outputs.undervoltage_warning = _protections.get_undervoltage_warning();
    _outputs.communication_loss_tripped = _protections.get_communication_loss_tripped();
    _outputs.communication_loss_alarmed = _protections.get_communication_loss_alarmed();
    _outputs.communication_loss_warning = _protections.get_communication_loss_warning();
    _outputs.door_open_tripped = _protections.get_door_open_tripped();
    _outputs.door_open_alarmed = _protections.get_door_open_alarmed();
}

void sfc_grid_side_protections_runner::run()
{
    // Handle manual reset request (for testing/commissioning)
    if (_parameters.force_reset_all) {
        reset();
        _parameters.force_reset_all = false;  // Clear flag after reset
    }

    const bool module_enabled = _parameters.protection_module_enabled;

    if (module_enabled) {
        _protections.update_communication_watchdog(_last_communication_valid);
        _protections.update_overcurrent_protection(_inputs.dc_link_current_pu);
        _protections.update_overvoltage_protection(_inputs.grid_voltage_magnitude_pu);
        _protections.update_undervoltage_protection(_inputs.grid_voltage_magnitude_pu);
    }

    _protections.resolve_protections(
        _inputs.door_is_open, module_enabled, static_cast<bool>(_parameters.confirm_alarms));

    _outputs.is_tripped = _protections.get_is_tripped();
    _outputs.is_inhibited = _protections.get_is_inhibited();
    _outputs.any_fault_active = _protections.get_any_fault_active();

    _outputs.overcurrent_tripped = _protections.get_overcurrent_tripped();
    _outputs.overcurrent_alarmed = _protections.get_overcurrent_alarmed();
    _outputs.overvoltage_tripped = _protections.get_overvoltage_tripped();
    _outputs.overvoltage_alarmed = _protections.get_overvoltage_alarmed();
    _outputs.undervoltage_tripped = _protections.get_undervoltage_tripped();
    _outputs.undervoltage_alarmed = _protections.get_undervoltage_alarmed();
    _outputs.overcurrent_warning = _protections.get_overcurrent_warning();
    _outputs.overvoltage_warning = _protections.get_overvoltage_warning();
    _outputs.undervoltage_warning = _protections.get_undervoltage_warning();
    _outputs.communication_loss_tripped = _protections.get_communication_loss_tripped();
    _outputs.communication_loss_alarmed = _protections.get_communication_loss_alarmed();
    _outputs.communication_loss_warning = _protections.get_communication_loss_warning();
    _outputs.door_open_tripped = _protections.get_door_open_tripped();
    _outputs.door_open_alarmed = _protections.get_door_open_alarmed();
}

void sfc_grid_side_protections_runner::stop() { reset(); }

void sfc_grid_side_protections_runner::update_communication_status(bool communication_valid)
{
    _last_communication_valid = communication_valid;
}

void sfc_grid_side_protections_runner::set_parameters()
{
    _protections.configure_overcurrent_is_trip(_parameters.overcurrent_severity == protection_severity::trip);
    _protections.configure_overvoltage_is_trip(_parameters.overvoltage_severity == protection_severity::trip);
    _protections.configure_undervoltage_is_trip(_parameters.undervoltage_severity == protection_severity::trip);
    _protections.configure_communication_loss_is_trip(
        _parameters.communication_loss_severity == protection_severity::trip);
    _protections.configure_door_open_is_trip(_parameters.door_open_severity == protection_severity::trip);

    _protections.configure_overcurrent_trip_threshold(_parameters.overcurrent_trip_threshold);
    _protections.configure_overcurrent_warning_threshold(_parameters.overcurrent_warning_threshold);
    _protections.configure_overcurrent_trip_delay(_parameters.overcurrent_trip_delay);
    _protections.configure_overcurrent_warning_delay(_parameters.overcurrent_warning_delay);

    _protections.configure_overvoltage_trip_threshold(_parameters.overvoltage_trip_threshold);
    _protections.configure_overvoltage_warning_threshold(_parameters.overvoltage_warning_threshold);
    _protections.configure_overvoltage_trip_delay(_parameters.overvoltage_trip_delay);
    _protections.configure_overvoltage_warning_delay(_parameters.overvoltage_warning_delay);

    _protections.configure_undervoltage_trip_threshold(_parameters.undervoltage_trip_threshold);
    _protections.configure_undervoltage_warning_threshold(_parameters.undervoltage_warning_threshold);
    _protections.configure_undervoltage_trip_delay(_parameters.undervoltage_trip_delay);
    _protections.configure_undervoltage_warning_delay(_parameters.undervoltage_warning_delay);

    _protections.configure_comm_timeout_duration(_parameters.comm_timeout);
    _protections.configure_comm_reset_duration(_parameters.comm_reset);
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
