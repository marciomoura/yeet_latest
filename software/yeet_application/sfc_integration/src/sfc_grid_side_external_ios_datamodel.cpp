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
    bool mv_breaker_is_closed,
    bool start_command,
    bool stop_command,
    bool mv_breaker_is_tripping,
    bool emergency_off,
    bool door_is_open,
    bool external_fault,
    bool fans_are_running)
{
    apply_digital_input_overrides(auxiliaries_are_on, mv_breaker_is_closed, start_command, stop_command,
        mv_breaker_is_tripping, emergency_off, door_is_open, external_fault, fans_are_running);
}

void sfc_grid_side_external_ios_datamodel::set_fiber_optics_rx(const fiber_optic::machine_to_grid_message& msg)
{
    _fiber_optics_inputs_from_machine.measured_speed.write(msg.measured_speed);
    _fiber_optics_inputs_from_machine.current_reference.write(msg.dc_link_current_reference);
    _fiber_optics_inputs_from_machine.machine_voltage_magnitude.write(msg.machine_voltage_magnitude);
    _fiber_optics_inputs_from_machine.request_force_commutation.write(msg.force_commutation_request);
    // Removed: shutdown_request is now a state machine parameter only, not transmitted via fiber optics
    _fiber_optics_inputs_from_machine.machine_protection_tripped.write(msg.machine_protection_tripped);
    _fiber_optics_inputs_from_machine.protection_inhibit.write(msg.protection_inhibit);
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
    return msg;
}

void sfc_grid_side_external_ios_datamodel::apply_digital_input_overrides(bool auxiliaries_are_on,
    bool mv_breaker_is_closed,
    bool start_command,
    bool stop_command,
    bool mv_breaker_is_tripping,
    bool emergency_off,
    bool door_is_open,
    bool external_fault,
    bool fans_are_running)
{
    // Helper lambda to apply polarity inversion
    auto apply_polarity = [](bool signal, io_polarity_t polarity) -> bool {
        return (polarity == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
    };

    // Apply override or pass-through for each signal individually, with polarity inversion
    bool aux_on = apply_polarity(auxiliaries_are_on, _parameters.di_auxiliaries_are_on_polarity.read());
    _digital_inputs.auxiliaries_are_on.write(_parameters.di_auxiliaries_are_on_override.read()
                                                 ? _parameters.di_auxiliaries_are_on_override_value.read()
                                                 : aux_on);

    bool breaker_closed = apply_polarity(mv_breaker_is_closed, _parameters.di_mv_breaker_is_closed_polarity.read());
    _digital_inputs.mv_breaker_is_closed.write(_parameters.di_mv_breaker_is_closed_override.read()
                                                   ? _parameters.di_mv_breaker_is_closed_override_value.read()
                                                   : breaker_closed);

    bool start_cmd = apply_polarity(start_command, _parameters.di_start_command_polarity.read());
    _digital_inputs.start_command.write(
        _parameters.di_start_command_override.read() ? _parameters.di_start_command_override_value.read() : start_cmd);

    bool stop_cmd = apply_polarity(stop_command, _parameters.di_stop_command_polarity.read());
    _digital_inputs.stop_command.write(
        _parameters.di_stop_command_override.read() ? _parameters.di_stop_command_override_value.read() : stop_cmd);

    bool breaker_trip = apply_polarity(mv_breaker_is_tripping, _parameters.di_mv_breaker_is_tripping_polarity.read());
    _digital_inputs.mv_breaker_is_tripping.write(_parameters.di_mv_breaker_is_tripping_override.read()
                                                     ? _parameters.di_mv_breaker_is_tripping_override_value.read()
                                                     : breaker_trip);

    bool emerg_off = apply_polarity(emergency_off, _parameters.di_emergency_off_polarity.read());
    _digital_inputs.emergency_off.write(
        _parameters.di_emergency_off_override.read() ? _parameters.di_emergency_off_override_value.read() : emerg_off);

    bool door_open = apply_polarity(door_is_open, _parameters.di_door_is_open_polarity.read());
    _digital_inputs.door_is_open.write(
        _parameters.di_door_is_open_override.read() ? _parameters.di_door_is_open_override_value.read() : door_open);

    bool ext_fault = apply_polarity(external_fault, _parameters.di_external_fault_polarity.read());
    _digital_inputs.external_fault.write(_parameters.di_external_fault_override.read()
                                             ? _parameters.di_external_fault_override_value.read()
                                             : ext_fault);

    bool fans_running = apply_polarity(fans_are_running, _parameters.di_fans_are_running_polarity.read());
    _digital_inputs.fans_are_running.write(_parameters.di_fans_are_running_override.read()
                                               ? _parameters.di_fans_are_running_override_value.read()
                                               : fans_running);
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

    // MV Breaker is closed
    _parameters.di_mv_breaker_is_closed_override.register_callback([this]() {
        if (_parameters.di_mv_breaker_is_closed_override.read()) {
            _digital_inputs.mv_breaker_is_closed.write(_parameters.di_mv_breaker_is_closed_override_value.read());
        }
    });
    _parameters.di_mv_breaker_is_closed_override_value.register_callback([this]() {
        if (_parameters.di_mv_breaker_is_closed_override.read()) {
            _digital_inputs.mv_breaker_is_closed.write(_parameters.di_mv_breaker_is_closed_override_value.read());
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

    // MV Breaker is tripping
    _parameters.di_mv_breaker_is_tripping_override.register_callback([this]() {
        if (_parameters.di_mv_breaker_is_tripping_override.read()) {
            _digital_inputs.mv_breaker_is_tripping.write(_parameters.di_mv_breaker_is_tripping_override_value.read());
        }
    });
    _parameters.di_mv_breaker_is_tripping_override_value.register_callback([this]() {
        if (_parameters.di_mv_breaker_is_tripping_override.read()) {
            _digital_inputs.mv_breaker_is_tripping.write(_parameters.di_mv_breaker_is_tripping_override_value.read());
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

    // External fault
    _parameters.di_external_fault_override.register_callback([this]() {
        if (_parameters.di_external_fault_override.read()) {
            _digital_inputs.external_fault.write(_parameters.di_external_fault_override_value.read());
        }
    });
    _parameters.di_external_fault_override_value.register_callback([this]() {
        if (_parameters.di_external_fault_override.read()) {
            _digital_inputs.external_fault.write(_parameters.di_external_fault_override_value.read());
        }
    });

    // Fans are running
    _parameters.di_fans_are_running_override.register_callback([this]() {
        if (_parameters.di_fans_are_running_override.read()) {
            _digital_inputs.fans_are_running.write(_parameters.di_fans_are_running_override_value.read());
        }
    });
    _parameters.di_fans_are_running_override_value.register_callback([this]() {
        if (_parameters.di_fans_are_running_override.read()) {
            _digital_inputs.fans_are_running.write(_parameters.di_fans_are_running_override_value.read());
        }
    });
}

void sfc_grid_side_external_ios_datamodel::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Digital input polarity configuration (ACTIVE_HIGH=0, ACTIVE_LOW=1)
    entry.parameters.insert(
        "di_auxiliaries_are_on_polarity", yeet::utils::make_enum_accessor(_parameters.di_auxiliaries_are_on_polarity));
    entry.parameters.insert("di_mv_breaker_is_closed_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_mv_breaker_is_closed_polarity));
    entry.parameters.insert(
        "di_start_command_polarity", yeet::utils::make_enum_accessor(_parameters.di_start_command_polarity));
    entry.parameters.insert(
        "di_stop_command_polarity", yeet::utils::make_enum_accessor(_parameters.di_stop_command_polarity));
    entry.parameters.insert("di_mv_breaker_is_tripping_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_mv_breaker_is_tripping_polarity));
    entry.parameters.insert(
        "di_emergency_off_polarity", yeet::utils::make_enum_accessor(_parameters.di_emergency_off_polarity));
    entry.parameters.insert(
        "di_door_is_open_polarity", yeet::utils::make_enum_accessor(_parameters.di_door_is_open_polarity));
    entry.parameters.insert(
        "di_external_fault_polarity", yeet::utils::make_enum_accessor(_parameters.di_external_fault_polarity));
    entry.parameters.insert(
        "di_fans_are_running_polarity", yeet::utils::make_enum_accessor(_parameters.di_fans_are_running_polarity));

    // Digital input override parameters (override flag + override value pairs)
    entry.parameters.insert(
        "di_auxiliaries_are_on_override", yeet::utils::make_bool_accessor(_parameters.di_auxiliaries_are_on_override));
    entry.parameters.insert("di_auxiliaries_are_on_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_auxiliaries_are_on_override_value));

    entry.parameters.insert("di_mv_breaker_is_closed_override",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_closed_override));
    entry.parameters.insert("di_mv_breaker_is_closed_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_closed_override_value));

    entry.parameters.insert(
        "di_start_command_override", yeet::utils::make_bool_accessor(_parameters.di_start_command_override));
    entry.parameters.insert("di_start_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_start_command_override_value));

    entry.parameters.insert(
        "di_stop_command_override", yeet::utils::make_bool_accessor(_parameters.di_stop_command_override));
    entry.parameters.insert(
        "di_stop_command_override_value", yeet::utils::make_bool_accessor(_parameters.di_stop_command_override_value));

    entry.parameters.insert("di_mv_breaker_is_tripping_override",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_tripping_override));
    entry.parameters.insert("di_mv_breaker_is_tripping_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_tripping_override_value));

    entry.parameters.insert(
        "di_emergency_off_override", yeet::utils::make_bool_accessor(_parameters.di_emergency_off_override));
    entry.parameters.insert("di_emergency_off_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_emergency_off_override_value));

    entry.parameters.insert(
        "di_door_is_open_override", yeet::utils::make_bool_accessor(_parameters.di_door_is_open_override));
    entry.parameters.insert(
        "di_door_is_open_override_value", yeet::utils::make_bool_accessor(_parameters.di_door_is_open_override_value));

    entry.parameters.insert(
        "di_external_fault_override", yeet::utils::make_bool_accessor(_parameters.di_external_fault_override));
    entry.parameters.insert("di_external_fault_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_external_fault_override_value));

    entry.parameters.insert(
        "di_fans_are_running_override", yeet::utils::make_bool_accessor(_parameters.di_fans_are_running_override));
    entry.parameters.insert("di_fans_are_running_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_fans_are_running_override_value));

    // Register the entry with the model registry
    registry.insert("grid_io_override", std::move(entry));

    // ==================== IO Index Mapping Parameters ====================
    // Keep IO index mapping parameters in a separate component so they don't mix
    // with override parameters in UIs.
    yeet::model_registry_entry io_entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Digital input mapping indices
    io_entry.parameters.insert(
        "di_auxiliaries_are_on_index", yeet::utils::make_units_accessor(_parameters.di_auxiliaries_are_on_index));
    io_entry.parameters.insert(
        "di_mv_breaker_is_closed_index", yeet::utils::make_units_accessor(_parameters.di_mv_breaker_is_closed_index));
    io_entry.parameters.insert(
        "di_start_command_index", yeet::utils::make_units_accessor(_parameters.di_start_command_index));
    io_entry.parameters.insert(
        "di_stop_command_index", yeet::utils::make_units_accessor(_parameters.di_stop_command_index));
    io_entry.parameters.insert("di_mv_breaker_is_tripping_index",
        yeet::utils::make_units_accessor(_parameters.di_mv_breaker_is_tripping_index));
    io_entry.parameters.insert(
        "di_emergency_off_index", yeet::utils::make_units_accessor(_parameters.di_emergency_off_index));
    io_entry.parameters.insert(
        "di_door_is_open_index", yeet::utils::make_units_accessor(_parameters.di_door_is_open_index));
    io_entry.parameters.insert(
        "di_external_fault_index", yeet::utils::make_units_accessor(_parameters.di_external_fault_index));
    io_entry.parameters.insert(
        "di_fans_are_running_index", yeet::utils::make_units_accessor(_parameters.di_fans_are_running_index));

    // Digital output mapping indices
    io_entry.parameters.insert(
        "do_mv_breaker_close_index", yeet::utils::make_units_accessor(_parameters.do_mv_breaker_close_index));
    io_entry.parameters.insert(
        "do_mv_breaker_open_index", yeet::utils::make_units_accessor(_parameters.do_mv_breaker_open_index));
    io_entry.parameters.insert(
        "do_on_command_acu_fan_1_index", yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_1_index));
    io_entry.parameters.insert(
        "do_on_command_acu_fan_2_index", yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_2_index));
    io_entry.parameters.insert(
        "do_on_command_acu_fan_3_index", yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_3_index));
    io_entry.parameters.insert(
        "do_on_command_acu_fan_4_index", yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_4_index));
    io_entry.parameters.insert(
        "do_software_trip_1_index", yeet::utils::make_units_accessor(_parameters.do_software_trip_1_index));
    io_entry.parameters.insert(
        "do_software_trip_2_index", yeet::utils::make_units_accessor(_parameters.do_software_trip_2_index));
    io_entry.parameters.insert(
        "do_software_trip_3_index", yeet::utils::make_units_accessor(_parameters.do_software_trip_3_index));

    // Digital output polarity configuration
    io_entry.parameters.insert(
        "do_mv_breaker_close_polarity", yeet::utils::make_enum_accessor(_parameters.do_mv_breaker_close_polarity));
    io_entry.parameters.insert(
        "do_mv_breaker_open_polarity", yeet::utils::make_enum_accessor(_parameters.do_mv_breaker_open_polarity));
    io_entry.parameters.insert("do_on_command_acu_fan_1_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_1_polarity));
    io_entry.parameters.insert("do_on_command_acu_fan_2_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_2_polarity));
    io_entry.parameters.insert("do_on_command_acu_fan_3_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_3_polarity));
    io_entry.parameters.insert("do_on_command_acu_fan_4_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_4_polarity));
    io_entry.parameters.insert(
        "do_software_trip_1_polarity", yeet::utils::make_enum_accessor(_parameters.do_software_trip_1_polarity));
    io_entry.parameters.insert(
        "do_software_trip_2_polarity", yeet::utils::make_enum_accessor(_parameters.do_software_trip_2_polarity));
    io_entry.parameters.insert(
        "do_software_trip_3_polarity", yeet::utils::make_enum_accessor(_parameters.do_software_trip_3_polarity));

    registry.insert("grid_io_configurations", std::move(io_entry));
}

bool sfc_grid_side_external_ios_datamodel::get_mv_breaker_close() const
{
    bool signal = _digital_outputs.mv_breaker_close.read();
    // Apply polarity inversion for output
    return (_parameters.do_mv_breaker_close_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_mv_breaker_open() const
{
    bool signal = _digital_outputs.mv_breaker_open.read();
    // Apply polarity inversion for output
    return (_parameters.do_mv_breaker_open_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_auxiliaries_on_command() const
{
    return _digital_outputs.auxiliaries_on_command.read();
}

// ACU fan ON command getters (all apply polarity inversion)
bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_1() const
{
    bool signal = _digital_outputs.on_command_acu_fan_1.read();
    return (_parameters.do_on_command_acu_fan_1_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_2() const
{
    bool signal = _digital_outputs.on_command_acu_fan_2.read();
    return (_parameters.do_on_command_acu_fan_2_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_3() const
{
    bool signal = _digital_outputs.on_command_acu_fan_3.read();
    return (_parameters.do_on_command_acu_fan_3_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_4() const
{
    bool signal = _digital_outputs.on_command_acu_fan_4.read();
    return (_parameters.do_on_command_acu_fan_4_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

// Software trip signal getters (all apply polarity inversion)
bool sfc_grid_side_external_ios_datamodel::get_software_trip_1() const
{
    bool signal = _digital_outputs.software_trip_1.read();
    return (_parameters.do_software_trip_1_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_software_trip_2() const
{
    bool signal = _digital_outputs.software_trip_2.read();
    return (_parameters.do_software_trip_2_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_software_trip_3() const
{
    bool signal = _digital_outputs.software_trip_3.read();
    return (_parameters.do_software_trip_3_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

}  // namespace yeet::application::sfc::integration
