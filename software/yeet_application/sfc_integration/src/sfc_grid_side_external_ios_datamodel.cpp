#include "sfc_integration/sfc_grid_side_external_ios_datamodel.hpp"

#include "yeet/model_registry.hpp"
#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_external_ios_datamodel::sfc_grid_side_external_ios_datamodel() { register_callbacks(); }

void sfc_grid_side_external_ios_datamodel::set_analog_inputs(yeet::control::adc_counts v_ab_counts,
    yeet::control::adc_counts v_bc_counts,
    yeet::control::adc_counts i_a_counts,
    yeet::control::adc_counts i_c_counts)
{
    _analog_inputs.v_ab_counts.write(v_ab_counts);
    _analog_inputs.v_bc_counts.write(v_bc_counts);
    _analog_inputs.i_a_counts.write(i_a_counts);
    _analog_inputs.i_c_counts.write(i_c_counts);
}

void sfc_grid_side_external_ios_datamodel::set_digital_inputs(
    bool auxiliaries_are_on, bool breakers_are_on, bool start_command, bool stop_command, bool shutdown_request)
{
    const bool digital_input_overrides_written = apply_digital_input_overrides();
    if (!digital_input_overrides_written) {
        _digital_inputs.auxiliaries_are_on.write(auxiliaries_are_on);
        _digital_inputs.breakers_are_on.write(breakers_are_on);
        _digital_inputs.start_command.write(start_command);
        _digital_inputs.stop_command.write(stop_command);
        _digital_inputs.shutdown_request.write(shutdown_request);
    }
}

void sfc_grid_side_external_ios_datamodel::set_fiber_optics_rx(const fiber_optic::machine_to_grid_message& msg)
{
    _fiber_optics_inputs_from_machine.measured_speed.write(msg.measured_speed);
    _fiber_optics_inputs_from_machine.current_reference.write(msg.dc_link_current_reference);
    _fiber_optics_inputs_from_machine.machine_voltage_magnitude.write(msg.machine_voltage_magnitude);
    _fiber_optics_inputs_from_machine.request_force_commutation.write(msg.force_commutation_request);
    _fiber_optics_inputs_from_machine.shutdown_request.write(msg.shutdown_request);
    _fiber_optics_inputs_from_machine.machine_protection_tripped.write(msg.machine_protection_tripped);
    _fiber_optics_inputs_from_machine.protection_inhibit.write(msg.protection_inhibit);
    _fiber_optics_inputs_from_machine.machine_trip_acknowledged.write(msg.machine_trip_acknowledged);
}

fiber_optic::grid_to_machine_message sfc_grid_side_external_ios_datamodel::get_fiber_optics_tx() const
{
    fiber_optic::grid_to_machine_message msg;
    msg.dc_link_current = _fiber_optics_outputs_to_machine.dc_link_current.read();
    msg.grid_voltage_magnitude = voltage_pu_t{0.0};  // TODO: Add grid voltage magnitude to outputs struct
    msg.is_forced_commutation =
        (_fiber_optics_outputs_to_machine.operating_mode.read() == sfc_operating_mode_t::forced_commutation);
    msg.force_commutation_acknowledged = _fiber_optics_outputs_to_machine.force_commutation_acknowledged.read();
    msg.release_pulses = _fiber_optics_outputs_to_machine.pulse_releases.read();
    msg.state_id = static_cast<std::uint8_t>(_fiber_optics_outputs_to_machine.current_state_id.read());
    msg.grid_release_protections = false;  // TODO: Add to outputs struct if needed
    msg.grid_trip_acknowledged = false;    // TODO: Add to outputs struct if needed
    return msg;
}

bool sfc_grid_side_external_ios_datamodel::apply_digital_input_overrides()
{
    if (!_parameters.override_digital_inputs.read()) {
        return false;
    }

    _digital_inputs.auxiliaries_are_on.write(_parameters.auxiliaries_are_on.read());
    _digital_inputs.breakers_are_on.write(_parameters.breakers_are_on.read());
    _digital_inputs.start_command.write(_parameters.start_command.read());
    _digital_inputs.stop_command.write(_parameters.stop_command.read());
    _digital_inputs.shutdown_request.write(_parameters.shutdown_request.read());

    return true;
}

void sfc_grid_side_external_ios_datamodel::register_callbacks()
{
    auto digital_inputs_callback = [this]() { apply_digital_input_overrides(); };

    _parameters.override_digital_inputs.register_callback(digital_inputs_callback);
    _parameters.auxiliaries_are_on.register_callback(digital_inputs_callback);
    _parameters.breakers_are_on.register_callback(digital_inputs_callback);
    _parameters.start_command.register_callback(digital_inputs_callback);
    _parameters.stop_command.register_callback(digital_inputs_callback);
    _parameters.shutdown_request.register_callback(digital_inputs_callback);
}

void sfc_grid_side_external_ios_datamodel::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Supervisory digital input override parameters
    entry.parameters.insert(
        "di_override_digital_inputs", yeet::utils::make_bool_accessor(_parameters.override_digital_inputs));
    entry.parameters.insert("di_auxiliaries_are_on", yeet::utils::make_bool_accessor(_parameters.auxiliaries_are_on));
    entry.parameters.insert("di_breakers_are_on", yeet::utils::make_bool_accessor(_parameters.breakers_are_on));
    entry.parameters.insert("di_start_command", yeet::utils::make_bool_accessor(_parameters.start_command));
    entry.parameters.insert("di_stop_command", yeet::utils::make_bool_accessor(_parameters.stop_command));
    entry.parameters.insert("di_shutdown_request", yeet::utils::make_bool_accessor(_parameters.shutdown_request));

    // Register the entry with the model registry
    registry.insert("grid_side_external_ios", std::move(entry));
}

}  // namespace yeet::application::sfc::integration
