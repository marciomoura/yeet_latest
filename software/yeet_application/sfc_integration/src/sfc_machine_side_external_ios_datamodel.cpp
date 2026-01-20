#include "sfc_integration/sfc_machine_side_external_ios_datamodel.hpp"

#include <cmath>

#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

sfc_machine_side_external_ios_datamodel::sfc_machine_side_external_ios_datamodel() { register_callbacks(); }

void sfc_machine_side_external_ios_datamodel::register_callbacks()
{
    // Digital input callbacks
    auto excitation_status_callback = [this]() {
        if (_parameters.di_excitation_status_override.read()) {
            _digital_inputs.excitation_status.write(_parameters.di_excitation_status_override_value.read());
        }
    };
    _parameters.di_excitation_status_override.register_callback(excitation_status_callback);
    _parameters.di_excitation_status_override_value.register_callback(excitation_status_callback);

    auto external_fault_callback = [this]() {
        if (_parameters.di_external_fault_override.read()) {
            _digital_inputs.external_fault.write(_parameters.di_external_fault_override_value.read());
        }
    };
    _parameters.di_external_fault_override.register_callback(external_fault_callback);
    _parameters.di_external_fault_override_value.register_callback(external_fault_callback);

    auto start_command_callback = [this]() {
        if (_parameters.di_start_command_override.read()) {
            _digital_inputs.start_command.write(_parameters.di_start_command_override_value.read());
        }
    };
    _parameters.di_start_command_override.register_callback(start_command_callback);
    _parameters.di_start_command_override_value.register_callback(start_command_callback);

    // Digital output callbacks - no action needed, overrides applied in getters
    auto digital_output_callback = [this]() { /* Overrides applied in getters */ };
    _parameters.do_excitation_on_command_override.register_callback(digital_output_callback);
    _parameters.do_excitation_on_command_override_value.register_callback(digital_output_callback);
    _parameters.do_excitation_off_command_override.register_callback(digital_output_callback);
    _parameters.do_excitation_off_command_override_value.register_callback(digital_output_callback);

    // Analog input callbacks
    auto excitation_current_actual_callback = [this]() {
        if (_parameters.ai_excitation_current_actual_override.read()) {
            _excitation_inputs.excitation_current_actual.write(
                _parameters.ai_excitation_current_actual_override_value.read());
        }
    };
    _parameters.ai_excitation_current_actual_override.register_callback(excitation_current_actual_callback);
    _parameters.ai_excitation_current_actual_override_value.register_callback(excitation_current_actual_callback);

    // Analog output callbacks - no action needed, overrides applied in getters
    auto analog_output_callback = [this]() { /* Overrides applied in getters */ };
    _parameters.ao_excitation_current_reference_override.register_callback(analog_output_callback);
    _parameters.ao_excitation_current_reference_override_value.register_callback(analog_output_callback);
}

// ==================== SIMPLIFIED SETTERS (Called from Embedded/Plant) ====================

void sfc_machine_side_external_ios_datamodel::set_analog_inputs(
    yeet::control::adc_counts v_ab_counts, yeet::control::adc_counts v_bc_counts)
{
    _analog_inputs.v_ab_counts.write(v_ab_counts);
    _analog_inputs.v_bc_counts.write(v_bc_counts);
}

void sfc_machine_side_external_ios_datamodel::set_digital_inputs(
    bool excitation_status, bool external_fault, bool start_command)
{
    // Lambda to apply polarity inversion
    auto apply_polarity = [](bool value, io_polarity_t polarity) -> bool {
        return (polarity == io_polarity_t::ACTIVE_LOW) ? !value : value;
    };

    // Apply polarity inversion to each digital input
    const bool excitation_status_inverted =
        apply_polarity(excitation_status, _parameters.di_excitation_status_polarity.read());
    const bool external_fault_inverted = apply_polarity(external_fault, _parameters.di_external_fault_polarity.read());
    const bool start_command_inverted = apply_polarity(start_command, _parameters.di_start_command_polarity.read());

    // Write to digital input signals (overrides will take precedence if enabled)
    if (!_parameters.di_excitation_status_override.read()) {
        _digital_inputs.excitation_status.write(excitation_status_inverted);
    }

    // External fault can be overridden for testing
    if (!_parameters.di_external_fault_override.read()) {
        _digital_inputs.external_fault.write(external_fault_inverted);
    }

    // Start command can be overridden for testing
    if (!_parameters.di_start_command_override.read()) {
        _digital_inputs.start_command.write(start_command_inverted);
    }
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
    _fiber_optics_inputs_from_grid.grid_release_protections.write(msg.grid_release_protections);
    _fiber_optics_inputs_from_grid.machine_release_protections.write(msg.machine_release_protections);
    _fiber_optics_inputs_from_grid.communication_ok.write(msg.communication_ok);
}

fiber_optic::machine_to_grid_message sfc_machine_side_external_ios_datamodel::get_fiber_optics_tx() const
{
    fiber_optic::machine_to_grid_message msg;
    msg.measured_speed = _fiber_optics_outputs_to_grid.measured_speed.read();
    msg.dc_link_current_reference = _fiber_optics_outputs_to_grid.dc_link_current_reference.read();
    msg.machine_voltage_magnitude = _fiber_optics_outputs_to_grid.machine_voltage_magnitude.read();
    msg.force_commutation_request = _fiber_optics_outputs_to_grid.force_commutation_request.read();
    msg.machine_protection_tripped = _fiber_optics_outputs_to_grid.machine_protection_tripped.read();
    msg.protection_inhibit = _fiber_optics_outputs_to_grid.protection_inhibit.read();
    msg.communication_ok = _fiber_optics_outputs_to_grid.communication_ok.read();
    return msg;
}

void sfc_machine_side_external_ios_datamodel::set_excitation_inputs(current_pu_t excitation_current_actual)
{
    if (!_parameters.ai_excitation_current_actual_override.read()) {
        _excitation_inputs.excitation_current_actual.write(excitation_current_actual);
    }
}

bool sfc_machine_side_external_ios_datamodel::get_excitation_on_command() const
{
    bool raw_value{};
    // Check if test override is active
    if (_parameters.do_excitation_on_command_override.read()) {
        raw_value = _parameters.do_excitation_on_command_override_value.read();
    }
    else {
        // Otherwise return the command from the excitation controller
        raw_value = _digital_outputs.excitation_on_command.read();
    }

    // Apply polarity inversion
    return (_parameters.do_excitation_on_command_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !raw_value : raw_value;
}

bool sfc_machine_side_external_ios_datamodel::get_excitation_off_command() const
{
    bool raw_value{};
    // Check if test override is active
    if (_parameters.do_excitation_off_command_override.read()) {
        raw_value = _parameters.do_excitation_off_command_override_value.read();
    }
    else {
        // Otherwise return the command from the excitation controller
        raw_value = _digital_outputs.excitation_off_command.read();
    }

    // Apply polarity inversion
    return (_parameters.do_excitation_off_command_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !raw_value
                                                                                                : raw_value;
}

current_pu_t sfc_machine_side_external_ios_datamodel::get_excitation_current_reference() const
{
    // Check if test override is active
    if (_parameters.ao_excitation_current_reference_override.read()) {
        return _parameters.ao_excitation_current_reference_override_value.read();
    }
    // Otherwise return the reference from the excitation controller
    return _excitation_outputs.excitation_current_reference.read();
}

control::six_pulse_generator::pulses sfc_machine_side_external_ios_datamodel::get_thyristor_pulse_outputs() const
{
    return _thyristor_output.firing_pulses.read();
}

// ==================== PRIVATE HELPER METHODS ====================

void sfc_machine_side_external_ios_datamodel::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    static constexpr std::string_view override_component = "machine_io_override";
    static constexpr std::string_view config_component = "machine_io_configurations";

    // Digital Input Overrides
    handler.register_parameter(override_component, "di_excitation_status_override",
        yeet::utils::make_bool_accessor(_parameters.di_excitation_status_override));
    handler.register_parameter(override_component, "di_excitation_status_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_excitation_status_override_value));
    handler.register_parameter(override_component, "di_external_fault_override",
        yeet::utils::make_bool_accessor(_parameters.di_external_fault_override));
    handler.register_parameter(override_component, "di_external_fault_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_external_fault_override_value));
    handler.register_parameter(override_component, "di_start_command_override",
        yeet::utils::make_bool_accessor(_parameters.di_start_command_override));
    handler.register_parameter(override_component, "di_start_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_start_command_override_value));

    // Digital Output Overrides
    handler.register_parameter(override_component, "do_excitation_on_command_override",
        yeet::utils::make_bool_accessor(_parameters.do_excitation_on_command_override));
    handler.register_parameter(override_component, "do_excitation_on_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_excitation_on_command_override_value));
    handler.register_parameter(override_component, "do_excitation_off_command_override",
        yeet::utils::make_bool_accessor(_parameters.do_excitation_off_command_override));
    handler.register_parameter(override_component, "do_excitation_off_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_excitation_off_command_override_value));

    // Analog Input Overrides
    handler.register_parameter(override_component, "ai_excitation_current_actual_override",
        yeet::utils::make_bool_accessor(_parameters.ai_excitation_current_actual_override));
    handler.register_parameter(override_component, "ai_excitation_current_actual_override_value",
        yeet::utils::make_units_accessor(_parameters.ai_excitation_current_actual_override_value));

    // Analog Output Overrides
    handler.register_parameter(override_component, "ao_excitation_current_reference_override",
        yeet::utils::make_bool_accessor(_parameters.ao_excitation_current_reference_override));
    handler.register_parameter(override_component, "ao_excitation_current_reference_override_value",
        yeet::utils::make_units_accessor(_parameters.ao_excitation_current_reference_override_value));

    // Analog Output Scaling Parameters
    handler.register_parameter(override_component, "ao_excitation_current_reference_gain",
        yeet::utils::make_units_accessor(_parameters.ao_excitation_current_reference_gain));
    handler.register_parameter(override_component, "ao_excitation_current_reference_offset",
        yeet::utils::make_units_accessor(_parameters.ao_excitation_current_reference_offset));
    handler.register_parameter(override_component, "ao_excitation_current_reference_minimum",
        yeet::utils::make_units_accessor(_parameters.ao_excitation_current_reference_minimum));
    handler.register_parameter(override_component, "ao_excitation_current_reference_maximum",
        yeet::utils::make_units_accessor(_parameters.ao_excitation_current_reference_maximum));

    // ==================== IO Index Mapping Parameters ====================
    // Keep IO index mapping parameters in a separate component so they don't mix
    // with excitation override parameters in UIs.

    // Digital input mapping indices
    handler.register_parameter(config_component, "di_excitation_status_index",
        yeet::utils::make_units_accessor(_parameters.di_excitation_status_index));
    handler.register_parameter(config_component, "di_external_fault_index",
        yeet::utils::make_units_accessor(_parameters.di_external_fault_index));
    handler.register_parameter(config_component, "di_start_command_index",
        yeet::utils::make_units_accessor(_parameters.di_start_command_index));

    // Digital input polarity
    handler.register_parameter(config_component, "di_excitation_status_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_excitation_status_polarity));
    handler.register_parameter(config_component, "di_external_fault_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_external_fault_polarity));
    handler.register_parameter(config_component, "di_start_command_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_start_command_polarity));

    // Digital output mapping indices
    handler.register_parameter(config_component, "do_excitation_on_command_index",
        yeet::utils::make_units_accessor(_parameters.do_excitation_on_command_index));
    handler.register_parameter(config_component, "do_excitation_off_command_index",
        yeet::utils::make_units_accessor(_parameters.do_excitation_off_command_index));

    // Digital output polarity
    handler.register_parameter(config_component, "do_excitation_on_command_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_excitation_on_command_polarity));
    handler.register_parameter(config_component, "do_excitation_off_command_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_excitation_off_command_polarity));

    // Analog output mapping indices
    handler.register_parameter(config_component, "ao_excitation_current_reference_index",
        yeet::utils::make_units_accessor(_parameters.ao_excitation_current_reference_index));
}

}  // namespace yeet::application::sfc::integration
