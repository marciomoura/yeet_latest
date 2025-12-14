#include "sfc_integration/sfc_machine_side_external_ios_datamodel.hpp"

#include <cmath>

#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

sfc_machine_side_external_ios_datamodel::sfc_machine_side_external_ios_datamodel() { register_callbacks(); }

void sfc_machine_side_external_ios_datamodel::register_callbacks()
{
    auto excitation_outputs_callback = [this]() { apply_excitation_overrides(); };
    auto excitation_inputs_callback = [this]() { apply_excitation_input_overrides(); };

    _parameters.override_excitation_outputs.register_callback(excitation_outputs_callback);
    _parameters.excitation_start_command.register_callback(excitation_outputs_callback);
    _parameters.excitation_stop_command.register_callback(excitation_outputs_callback);
    _parameters.excitation_current_reference.register_callback(excitation_outputs_callback);

    _parameters.override_excitation_inputs.register_callback(excitation_inputs_callback);
    _parameters.excitation_current_actual.register_callback(excitation_inputs_callback);
    _parameters.excitation_running.register_callback(excitation_inputs_callback);
}

// ==================== SIMPLIFIED SETTERS (Called from Embedded/Plant) ====================

void sfc_machine_side_external_ios_datamodel::set_analog_inputs(
    yeet::control::adc_counts v_ab_counts, yeet::control::adc_counts v_bc_counts)
{
    _analog_inputs.v_ab_counts.write(v_ab_counts);
    _analog_inputs.v_bc_counts.write(v_bc_counts);
}

void sfc_machine_side_external_ios_datamodel::set_fiber_optics_rx(const fiber_optic::grid_to_machine_message& msg)
{
    // Convert boolean is_forced_commutation to operating_mode enum
    sfc_operating_mode_t operating_mode = msg.is_forced_commutation ? sfc_operating_mode_t::forced_commutation
                                                                    : sfc_operating_mode_t::natural_commutation;

    // Write all fiber optic inputs from the decoded message
    _fiber_optics_inputs_from_grid.operating_mode.write(operating_mode);
    _fiber_optics_inputs_from_grid.current_state_id.write(static_cast<sfc_state_id>(msg.state_id));
    _fiber_optics_inputs_from_grid.force_commutation_acknowledged.write(msg.force_commutation_acknowledged);
    _fiber_optics_inputs_from_grid.release_pulses.write(msg.release_pulses);
    _fiber_optics_inputs_from_grid.dc_link_current.write(msg.dc_link_current);
    _fiber_optics_inputs_from_grid.grid_voltage_magnitude.write(msg.grid_voltage_magnitude);
    _fiber_optics_inputs_from_grid.grid_release_protections.write(msg.grid_release_protections);
    _fiber_optics_inputs_from_grid.grid_trip_acknowledged.write(msg.grid_trip_acknowledged);
}

fiber_optic::machine_to_grid_message sfc_machine_side_external_ios_datamodel::get_fiber_optics_tx() const
{
    fiber_optic::machine_to_grid_message msg;
    msg.measured_speed = _fiber_optics_outputs_to_grid.measured_speed.read();
    msg.dc_link_current_reference = _fiber_optics_outputs_to_grid.dc_link_current_reference.read();
    msg.machine_voltage_magnitude = _fiber_optics_outputs_to_grid.machine_voltage_magnitude.read();
    msg.force_commutation_request = _fiber_optics_outputs_to_grid.force_commutation_request.read();
    msg.shutdown_request = _fiber_optics_outputs_to_grid.shutdown_request.read();
    msg.machine_protection_tripped = _fiber_optics_outputs_to_grid.machine_protection_tripped.read();
    msg.protection_inhibit = _fiber_optics_outputs_to_grid.protection_inhibit.read();
    msg.machine_trip_acknowledged = _fiber_optics_outputs_to_grid.machine_trip_acknowledged.read();
    return msg;
}

void sfc_machine_side_external_ios_datamodel::set_excitation_inputs(
    current_pu_t excitation_current_actual, bool excitation_running)
{
    const bool excitation_input_overrides_written = apply_excitation_input_overrides();
    if (!excitation_input_overrides_written) {
        _excitation_inputs.excitation_current_actual.write(excitation_current_actual);
        _excitation_inputs.excitation_running.write(excitation_running);
    }
}

bool sfc_machine_side_external_ios_datamodel::get_excitation_release_command() const
{
    // Check if test override is active
    if (_parameters.override_excitation_outputs.read()) {
        return _parameters.excitation_start_command.read();  // Re-use start command for release
    }
    // Otherwise return the command from the excitation controller
    return _excitation_outputs.excitation_release.read();
}

current_pu_t sfc_machine_side_external_ios_datamodel::get_excitation_current_reference() const
{
    // Check if test override is active
    if (_parameters.override_excitation_outputs.read()) {
        return _parameters.excitation_current_reference.read();
    }
    // Otherwise return the reference from the excitation controller
    return _excitation_outputs.excitation_current_reference.read();
}

control::six_pulse_generator::pulses sfc_machine_side_external_ios_datamodel::get_thyristor_pulse_outputs() const
{
    return _thyristor_output.firing_pulses.read();
}

// ==================== PRIVATE HELPER METHODS ====================

bool sfc_machine_side_external_ios_datamodel::apply_excitation_overrides()
{
    // This is a no-op since overrides are applied in the getters
    // Kept for callback registration consistency
    return _parameters.override_excitation_outputs.read();
}

bool sfc_machine_side_external_ios_datamodel::apply_excitation_input_overrides()
{
    if (!_parameters.override_excitation_inputs.read()) {
        return false;
    }

    _excitation_inputs.excitation_current_actual.write(_parameters.excitation_current_actual.read());
    _excitation_inputs.excitation_running.write(_parameters.excitation_running.read());

    return true;
}

void sfc_machine_side_external_ios_datamodel::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    entry.parameters.insert(
        "exc_override_excitation_outputs", yeet::utils::make_bool_accessor(_parameters.override_excitation_outputs));
    entry.parameters.insert(
        "exc_excitation_start_command", yeet::utils::make_bool_accessor(_parameters.excitation_start_command));
    entry.parameters.insert(
        "exc_excitation_stop_command", yeet::utils::make_bool_accessor(_parameters.excitation_stop_command));
    entry.parameters.insert(
        "exc_excitation_current_reference", yeet::utils::make_units_accessor(_parameters.excitation_current_reference));
    entry.parameters.insert(
        "exc_override_excitation_inputs", yeet::utils::make_bool_accessor(_parameters.override_excitation_inputs));
    entry.parameters.insert(
        "exc_excitation_current_actual", yeet::utils::make_units_accessor(_parameters.excitation_current_actual));
    entry.parameters.insert("exc_excitation_running", yeet::utils::make_bool_accessor(_parameters.excitation_running));

    registry.insert("machine_side_external_ios", std::move(entry));

    // ==================== IO Index Mapping Parameters ====================
    // Keep IO index mapping parameters in a separate component so they don't mix
    // with excitation override parameters in UIs.
    yeet::model_registry_entry io_entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Digital input mapping indices
    io_entry.parameters.insert(
        "di_excitation_running_index", yeet::utils::make_units_accessor(_parameters.di_excitation_running_index));

    // Digital output mapping indices
    io_entry.parameters.insert(
        "do_fiber_link_ok_index", yeet::utils::make_units_accessor(_parameters.do_fiber_link_ok_index));
    io_entry.parameters.insert(
        "do_excitation_release_index", yeet::utils::make_units_accessor(_parameters.do_excitation_release_index));

    // Analog output mapping indices
    io_entry.parameters.insert("ao_excitation_current_reference_index",
        yeet::utils::make_units_accessor(_parameters.ao_excitation_current_reference_index));

    registry.insert("machine_io_indexes", std::move(io_entry));
}

}  // namespace yeet::application::sfc::integration
