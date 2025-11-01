#include "sfc_integration/sfc_grid_side_external_ios_datamodel.hpp"

#include "yeet/model_registry.hpp"
#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_external_ios_datamodel::sfc_grid_side_external_ios_datamodel() { register_callbacks(); }

void sfc_grid_side_external_ios_datamodel::update_analog_inputs(yeet::control::adc_counts v_ab_counts,
    yeet::control::adc_counts v_bc_counts,
    yeet::control::adc_counts i_a_counts,
    yeet::control::adc_counts i_c_counts)
{
    _analog_inputs.v_ab_counts.write(v_ab_counts);
    _analog_inputs.v_bc_counts.write(v_bc_counts);
    _analog_inputs.i_a_counts.write(i_a_counts);
    _analog_inputs.i_c_counts.write(i_c_counts);
}

void sfc_grid_side_external_ios_datamodel::update_digital_inputs(
    bool auxiliaries_are_on, bool breakers_are_on, bool start_command, bool stop_command, bool shutdown_request)
{
    const bool digital_input_overrides_written = write_digital_input_overrides();
    if (!digital_input_overrides_written) {
        _digital_inputs.auxiliaries_are_on.write(auxiliaries_are_on);
        _digital_inputs.breakers_are_on.write(breakers_are_on);
        _digital_inputs.start_command.write(start_command);
        _digital_inputs.stop_command.write(stop_command);
        _digital_inputs.shutdown_request.write(shutdown_request);
    }
}

void sfc_grid_side_external_ios_datamodel::update_fiber_optics_outputs_to_machine(sfc_operating_mode_t operating_mode,
    bool force_commutation_acknowledged,
    bool is_current_zero,
    bool current_controller_is_saturated,
    bool pulse_releases,
    current_pu_t dc_link_current,
    bool freeze_integrator,
    bool is_requesting_main_pi_reset,
    yeet::application::sfc::sfc_state_id current_state_id)
{
    // Fiber-optics outputs are always written (not overrideable)
    // These are primary outputs from grid-side internals
    _fiber_optics_outputs_to_machine.operating_mode.write(operating_mode);
    _fiber_optics_outputs_to_machine.force_commutation_acknowledged.write(force_commutation_acknowledged);
    _fiber_optics_outputs_to_machine.is_current_zero.write(is_current_zero);
    _fiber_optics_outputs_to_machine.current_controller_is_saturated.write(current_controller_is_saturated);
    _fiber_optics_outputs_to_machine.pulse_releases.write(pulse_releases);
    _fiber_optics_outputs_to_machine.dc_link_current.write(dc_link_current);
    _fiber_optics_outputs_to_machine.freeze_integrator.write(freeze_integrator);
    _fiber_optics_outputs_to_machine.is_requesting_main_pi_reset.write(is_requesting_main_pi_reset);
    _fiber_optics_outputs_to_machine.current_state_id.write(current_state_id);
}

void sfc_grid_side_external_ios_datamodel::update_fiber_optics_inputs_from_machine(speed_pu_t measured_speed_pu,
    current_pu_t dc_link_current_reference,
    voltage_pu_t machine_voltage_magnitude,
    bool request_force_commutation)
{
    const bool machine_side_overrides_written = write_machine_side_fiber_optics_input_overrides();
    if (!machine_side_overrides_written) {
        _fiber_optics_inputs_from_machine.measured_speed_pu.write(measured_speed_pu);
        _fiber_optics_inputs_from_machine.dc_link_current_reference.write(dc_link_current_reference);
        _fiber_optics_inputs_from_machine.machine_voltage_magnitude.write(machine_voltage_magnitude);
        _fiber_optics_inputs_from_machine.request_force_commutation.write(request_force_commutation);
    }
}

bool sfc_grid_side_external_ios_datamodel::write_digital_input_overrides()
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

bool sfc_grid_side_external_ios_datamodel::write_machine_side_fiber_optics_input_overrides()
{
    if (!_parameters.override_machine_side_fiber_optics_inputs.read()) {
        return false;
    }

    _fiber_optics_inputs_from_machine.measured_speed_pu.write(_parameters.measured_speed_pu.read());
    _fiber_optics_inputs_from_machine.dc_link_current_reference.write(_parameters.dc_link_current_reference.read());
    _fiber_optics_inputs_from_machine.machine_voltage_magnitude.write(_parameters.machine_voltage_magnitude.read());
    _fiber_optics_inputs_from_machine.request_force_commutation.write(_parameters.request_force_commutation.read());

    return true;
}

void sfc_grid_side_external_ios_datamodel::register_callbacks()
{
    auto digital_inputs_callback = [this]() { write_digital_input_overrides(); };

    auto machine_side_fiber_optics_inputs_callback = [this]() { write_machine_side_fiber_optics_input_overrides(); };

    _parameters.override_digital_inputs.register_callback(digital_inputs_callback);
    _parameters.auxiliaries_are_on.register_callback(digital_inputs_callback);
    _parameters.breakers_are_on.register_callback(digital_inputs_callback);
    _parameters.start_command.register_callback(digital_inputs_callback);
    _parameters.stop_command.register_callback(digital_inputs_callback);
    _parameters.shutdown_request.register_callback(digital_inputs_callback);

    _parameters.override_machine_side_fiber_optics_inputs.register_callback(machine_side_fiber_optics_inputs_callback);
    _parameters.measured_speed_pu.register_callback(machine_side_fiber_optics_inputs_callback);
    _parameters.dc_link_current_reference.register_callback(machine_side_fiber_optics_inputs_callback);
    _parameters.machine_voltage_magnitude.register_callback(machine_side_fiber_optics_inputs_callback);
    _parameters.request_force_commutation.register_callback(machine_side_fiber_optics_inputs_callback);
}

void sfc_grid_side_external_ios_datamodel::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Supervisory digital input override parameters
    entry.parameters.insert(
        "di_override_digital_inputs", yeet::utils::make_units_accessor(_parameters.override_digital_inputs));
    entry.parameters.insert("di_auxiliaries_are_on", yeet::utils::make_units_accessor(_parameters.auxiliaries_are_on));
    entry.parameters.insert("di_breakers_are_on", yeet::utils::make_units_accessor(_parameters.breakers_are_on));
    entry.parameters.insert("di_start_command", yeet::utils::make_units_accessor(_parameters.start_command));
    entry.parameters.insert("di_stop_command", yeet::utils::make_units_accessor(_parameters.stop_command));
    entry.parameters.insert("di_shutdown_request", yeet::utils::make_units_accessor(_parameters.shutdown_request));

    // Machine-side fiber-optics input override parameters
    entry.parameters.insert("fo_override_machine_side_inputs",
        yeet::utils::make_units_accessor(_parameters.override_machine_side_fiber_optics_inputs));
    entry.parameters.insert("fo_measured_speed_pu", yeet::utils::make_units_accessor(_parameters.measured_speed_pu));
    entry.parameters.insert(
        "fo_dc_link_current_reference", yeet::utils::make_units_accessor(_parameters.dc_link_current_reference));
    entry.parameters.insert(
        "fo_machine_voltage_magnitude", yeet::utils::make_units_accessor(_parameters.machine_voltage_magnitude));
    entry.parameters.insert(
        "fo_request_force_commutation", yeet::utils::make_units_accessor(_parameters.request_force_commutation));

    // Register the entry with the model registry
    registry.insert("grid_side_external_ios", std::move(entry));
}

}  // namespace yeet::application::sfc::integration
