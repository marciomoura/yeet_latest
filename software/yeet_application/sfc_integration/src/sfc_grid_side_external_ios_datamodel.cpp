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

void sfc_grid_side_external_ios_datamodel::set_digital_inputs(bool auxiliaries_are_on,
    bool mcb_is_closed,
    bool start_command,
    bool stop_command,
    bool shutdown_request,
    bool breaker_is_closed,
    bool breaker_is_tripping,
    bool emergency_off,
    bool door_is_open)
{
    apply_digital_input_overrides(auxiliaries_are_on, mcb_is_closed, start_command, stop_command, shutdown_request,
        breaker_is_closed, breaker_is_tripping, emergency_off, door_is_open);
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

void sfc_grid_side_external_ios_datamodel::apply_digital_input_overrides(bool auxiliaries_are_on,
    bool mcb_is_closed,
    bool start_command,
    bool stop_command,
    bool shutdown_request,
    bool breaker_is_closed,
    bool breaker_is_tripping,
    bool emergency_off,
    bool door_is_open)
{
    // Apply override or pass-through for each signal individually
    _digital_inputs.auxiliaries_are_on.write(_parameters.di_auxiliaries_are_on_override.read()
                                                 ? _parameters.di_auxiliaries_are_on_override_value.read()
                                                 : auxiliaries_are_on);

    _digital_inputs.mcb_is_closed.write(_parameters.di_mcb_is_closed_override.read()
                                            ? _parameters.di_mcb_is_closed_override_value.read()
                                            : mcb_is_closed);

    _digital_inputs.start_command.write(_parameters.di_start_command_override.read()
                                            ? _parameters.di_start_command_override_value.read()
                                            : start_command);

    _digital_inputs.stop_command.write(
        _parameters.di_stop_command_override.read() ? _parameters.di_stop_command_override_value.read() : stop_command);

    _digital_inputs.shutdown_request.write(_parameters.di_shutdown_request_override.read()
                                               ? _parameters.di_shutdown_request_override_value.read()
                                               : shutdown_request);

    _digital_inputs.breaker_is_closed.write(_parameters.di_breaker_is_closed_override.read()
                                                ? _parameters.di_breaker_is_closed_override_value.read()
                                                : breaker_is_closed);

    _digital_inputs.breaker_is_tripping.write(_parameters.di_breaker_is_tripping_override.read()
                                                  ? _parameters.di_breaker_is_tripping_override_value.read()
                                                  : breaker_is_tripping);

    _digital_inputs.emergency_off.write(_parameters.di_emergency_off_override.read()
                                            ? _parameters.di_emergency_off_override_value.read()
                                            : emergency_off);

    _digital_inputs.door_is_open.write(
        _parameters.di_door_is_open_override.read() ? _parameters.di_door_is_open_override_value.read() : door_is_open);
}

void sfc_grid_side_external_ios_datamodel::register_callbacks()
{
    // Callbacks for simulation/test mode: apply override directly to digital input signal
    // when either the override flag or the override value changes.
    // In embedded mode, set_digital_inputs() handles this during each control cycle.

    // Auxiliaries are on
    _parameters.di_auxiliaries_are_on_override.register_callback([this]() {
        if (_parameters.di_auxiliaries_are_on_override.read()) {
            _digital_inputs.auxiliaries_are_on.write(_parameters.di_auxiliaries_are_on_override_value.read());
        }
    });
    _parameters.di_auxiliaries_are_on_override_value.register_callback([this]() {
        if (_parameters.di_auxiliaries_are_on_override.read()) {
            _digital_inputs.auxiliaries_are_on.write(_parameters.di_auxiliaries_are_on_override_value.read());
        }
    });

    // MCB is closed
    _parameters.di_mcb_is_closed_override.register_callback([this]() {
        if (_parameters.di_mcb_is_closed_override.read()) {
            _digital_inputs.mcb_is_closed.write(_parameters.di_mcb_is_closed_override_value.read());
        }
    });
    _parameters.di_mcb_is_closed_override_value.register_callback([this]() {
        if (_parameters.di_mcb_is_closed_override.read()) {
            _digital_inputs.mcb_is_closed.write(_parameters.di_mcb_is_closed_override_value.read());
        }
    });

    // Start command
    _parameters.di_start_command_override.register_callback([this]() {
        if (_parameters.di_start_command_override.read()) {
            _digital_inputs.start_command.write(_parameters.di_start_command_override_value.read());
        }
    });
    _parameters.di_start_command_override_value.register_callback([this]() {
        if (_parameters.di_start_command_override.read()) {
            _digital_inputs.start_command.write(_parameters.di_start_command_override_value.read());
        }
    });

    // Stop command
    _parameters.di_stop_command_override.register_callback([this]() {
        if (_parameters.di_stop_command_override.read()) {
            _digital_inputs.stop_command.write(_parameters.di_stop_command_override_value.read());
        }
    });
    _parameters.di_stop_command_override_value.register_callback([this]() {
        if (_parameters.di_stop_command_override.read()) {
            _digital_inputs.stop_command.write(_parameters.di_stop_command_override_value.read());
        }
    });

    // Shutdown request
    _parameters.di_shutdown_request_override.register_callback([this]() {
        if (_parameters.di_shutdown_request_override.read()) {
            _digital_inputs.shutdown_request.write(_parameters.di_shutdown_request_override_value.read());
        }
    });
    _parameters.di_shutdown_request_override_value.register_callback([this]() {
        if (_parameters.di_shutdown_request_override.read()) {
            _digital_inputs.shutdown_request.write(_parameters.di_shutdown_request_override_value.read());
        }
    });

    // Breaker is closed
    _parameters.di_breaker_is_closed_override.register_callback([this]() {
        if (_parameters.di_breaker_is_closed_override.read()) {
            _digital_inputs.breaker_is_closed.write(_parameters.di_breaker_is_closed_override_value.read());
        }
    });
    _parameters.di_breaker_is_closed_override_value.register_callback([this]() {
        if (_parameters.di_breaker_is_closed_override.read()) {
            _digital_inputs.breaker_is_closed.write(_parameters.di_breaker_is_closed_override_value.read());
        }
    });

    // Breaker is tripping
    _parameters.di_breaker_is_tripping_override.register_callback([this]() {
        if (_parameters.di_breaker_is_tripping_override.read()) {
            _digital_inputs.breaker_is_tripping.write(_parameters.di_breaker_is_tripping_override_value.read());
        }
    });
    _parameters.di_breaker_is_tripping_override_value.register_callback([this]() {
        if (_parameters.di_breaker_is_tripping_override.read()) {
            _digital_inputs.breaker_is_tripping.write(_parameters.di_breaker_is_tripping_override_value.read());
        }
    });

    // Emergency off
    _parameters.di_emergency_off_override.register_callback([this]() {
        if (_parameters.di_emergency_off_override.read()) {
            _digital_inputs.emergency_off.write(_parameters.di_emergency_off_override_value.read());
        }
    });
    _parameters.di_emergency_off_override_value.register_callback([this]() {
        if (_parameters.di_emergency_off_override.read()) {
            _digital_inputs.emergency_off.write(_parameters.di_emergency_off_override_value.read());
        }
    });

    // Door is open
    _parameters.di_door_is_open_override.register_callback([this]() {
        if (_parameters.di_door_is_open_override.read()) {
            _digital_inputs.door_is_open.write(_parameters.di_door_is_open_override_value.read());
        }
    });
    _parameters.di_door_is_open_override_value.register_callback([this]() {
        if (_parameters.di_door_is_open_override.read()) {
            _digital_inputs.door_is_open.write(_parameters.di_door_is_open_override_value.read());
        }
    });
}

void sfc_grid_side_external_ios_datamodel::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Digital input override parameters (override flag + override value pairs)
    entry.parameters.insert(
        "di_auxiliaries_are_on_override", yeet::utils::make_bool_accessor(_parameters.di_auxiliaries_are_on_override));
    entry.parameters.insert("di_auxiliaries_are_on_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_auxiliaries_are_on_override_value));

    entry.parameters.insert(
        "di_mcb_is_closed_override", yeet::utils::make_bool_accessor(_parameters.di_mcb_is_closed_override));
    entry.parameters.insert("di_mcb_is_closed_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_mcb_is_closed_override_value));

    entry.parameters.insert(
        "di_start_command_override", yeet::utils::make_bool_accessor(_parameters.di_start_command_override));
    entry.parameters.insert("di_start_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_start_command_override_value));

    entry.parameters.insert(
        "di_stop_command_override", yeet::utils::make_bool_accessor(_parameters.di_stop_command_override));
    entry.parameters.insert(
        "di_stop_command_override_value", yeet::utils::make_bool_accessor(_parameters.di_stop_command_override_value));

    entry.parameters.insert(
        "di_shutdown_request_override", yeet::utils::make_bool_accessor(_parameters.di_shutdown_request_override));
    entry.parameters.insert("di_shutdown_request_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_shutdown_request_override_value));

    entry.parameters.insert(
        "di_breaker_is_closed_override", yeet::utils::make_bool_accessor(_parameters.di_breaker_is_closed_override));
    entry.parameters.insert("di_breaker_is_closed_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_breaker_is_closed_override_value));

    entry.parameters.insert("di_breaker_is_tripping_override",
        yeet::utils::make_bool_accessor(_parameters.di_breaker_is_tripping_override));
    entry.parameters.insert("di_breaker_is_tripping_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_breaker_is_tripping_override_value));

    entry.parameters.insert(
        "di_emergency_off_override", yeet::utils::make_bool_accessor(_parameters.di_emergency_off_override));
    entry.parameters.insert("di_emergency_off_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_emergency_off_override_value));

    entry.parameters.insert(
        "di_door_is_open_override", yeet::utils::make_bool_accessor(_parameters.di_door_is_open_override));
    entry.parameters.insert(
        "di_door_is_open_override_value", yeet::utils::make_bool_accessor(_parameters.di_door_is_open_override_value));

    // Register the entry with the model registry
    registry.insert("grid_side_external_ios", std::move(entry));
}

bool sfc_grid_side_external_ios_datamodel::get_mcb_close() const { return _digital_outputs.mcb_close.read(); }

bool sfc_grid_side_external_ios_datamodel::get_mcb_open() const { return _digital_outputs.mcb_open.read(); }

}  // namespace yeet::application::sfc::integration
