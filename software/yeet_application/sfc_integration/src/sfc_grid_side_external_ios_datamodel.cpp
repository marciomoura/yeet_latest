#include "sfc_integration/sfc_grid_side_external_ios_datamodel.hpp"

#include "yeet/parameter_handler.hpp"
#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_external_ios_datamodel::sfc_grid_side_external_ios_datamodel()
{
    register_callbacks();

    // Configure debouncers with default 10ms delay and 1ms sampling time
    const real_t sampling_time = 0.001f;
    const real_t debounce_time = 0.010f;

    _di_auxiliaries_are_on_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_auxiliaries_are_on_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_mv_breaker_is_closed_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_mv_breaker_is_closed_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_start_command_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_start_command_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_stop_command_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_stop_command_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_mv_breaker_is_tripping_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_mv_breaker_is_tripping_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_emergency_off_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_emergency_off_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_door_is_open_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_door_is_open_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_external_fault_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_external_fault_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});

    _di_fans_are_running_debouncer.configure_sampling_time(duration_t{sampling_time});
    _di_fans_are_running_debouncer.configure_delay(duration_t{debounce_time}, duration_t{debounce_time});
}

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
    _fiber_optics_inputs_from_machine.communication_ok.write(msg.communication_ok);
}

fiber_optic::grid_to_machine_message sfc_grid_side_external_ios_datamodel::get_fiber_optics_tx() const
{
    fiber_optic::grid_to_machine_message msg;
    msg.dc_link_current = _fiber_optics_outputs_to_machine.dc_link_current.read();
    msg.is_forced_commutation =
        (_fiber_optics_outputs_to_machine.operating_mode.read() == sfc_operating_mode_t::forced_commutation);
    msg.force_commutation_acknowledged = _fiber_optics_outputs_to_machine.force_commutation_acknowledged.read();
    msg.release_pulses = _fiber_optics_outputs_to_machine.pulse_releases.read();
    msg.state_id = static_cast<std::uint8_t>(_fiber_optics_outputs_to_machine.current_state_id.read());
    msg.grid_release_protections = _fiber_optics_outputs_to_machine.grid_release_protections.read();
    msg.machine_release_protections = _fiber_optics_outputs_to_machine.machine_release_protections.read();
    msg.communication_ok = _fiber_optics_outputs_to_machine.communication_ok.read();
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

    // Apply override or pass-through for each signal individually, with polarity inversion and debouncing
    bool aux_on_raw = apply_polarity(auxiliaries_are_on, _parameters.di_auxiliaries_are_on_polarity.read());
    _di_auxiliaries_are_on_debouncer.update(aux_on_raw);
    _digital_inputs.auxiliaries_are_on.write(_parameters.di_auxiliaries_are_on_override.read()
                                                 ? _parameters.di_auxiliaries_are_on_override_value.read()
                                                 : _di_auxiliaries_are_on_debouncer.get_output());

    bool breaker_closed_raw = apply_polarity(mv_breaker_is_closed, _parameters.di_mv_breaker_is_closed_polarity.read());
    _di_mv_breaker_is_closed_debouncer.update(breaker_closed_raw);
    _digital_inputs.mv_breaker_is_closed.write(_parameters.di_mv_breaker_is_closed_override.read()
                                                   ? _parameters.di_mv_breaker_is_closed_override_value.read()
                                                   : _di_mv_breaker_is_closed_debouncer.get_output());

    bool start_cmd_raw = apply_polarity(start_command, _parameters.di_start_command_polarity.read());
    _di_start_command_debouncer.update(start_cmd_raw);
    _digital_inputs.start_command.write(_parameters.di_start_command_override.read()
                                            ? _parameters.di_start_command_override_value.read()
                                            : _di_start_command_debouncer.get_output());

    bool stop_cmd_raw = apply_polarity(stop_command, _parameters.di_stop_command_polarity.read());
    _di_stop_command_debouncer.update(stop_cmd_raw);
    _digital_inputs.stop_command.write(_parameters.di_stop_command_override.read()
                                           ? _parameters.di_stop_command_override_value.read()
                                           : _di_stop_command_debouncer.get_output());

    bool breaker_trip_raw =
        apply_polarity(mv_breaker_is_tripping, _parameters.di_mv_breaker_is_tripping_polarity.read());
    _di_mv_breaker_is_tripping_debouncer.update(breaker_trip_raw);
    _digital_inputs.mv_breaker_is_tripping.write(_parameters.di_mv_breaker_is_tripping_override.read()
                                                     ? _parameters.di_mv_breaker_is_tripping_override_value.read()
                                                     : _di_mv_breaker_is_tripping_debouncer.get_output());

    bool emerg_off_raw = apply_polarity(emergency_off, _parameters.di_emergency_off_polarity.read());
    _di_emergency_off_debouncer.update(emerg_off_raw);
    _digital_inputs.emergency_off.write(_parameters.di_emergency_off_override.read()
                                            ? _parameters.di_emergency_off_override_value.read()
                                            : _di_emergency_off_debouncer.get_output());

    bool door_open_raw = apply_polarity(door_is_open, _parameters.di_door_is_open_polarity.read());
    _di_door_is_open_debouncer.update(door_open_raw);
    _digital_inputs.door_is_open.write(_parameters.di_door_is_open_override.read()
                                           ? _parameters.di_door_is_open_override_value.read()
                                           : _di_door_is_open_debouncer.get_output());

    bool ext_fault_raw = apply_polarity(external_fault, _parameters.di_external_fault_polarity.read());
    _di_external_fault_debouncer.update(ext_fault_raw);
    _digital_inputs.external_fault.write(_parameters.di_external_fault_override.read()
                                             ? _parameters.di_external_fault_override_value.read()
                                             : _di_external_fault_debouncer.get_output());

    bool fans_running_raw = apply_polarity(fans_are_running, _parameters.di_fans_are_running_polarity.read());
    _di_fans_are_running_debouncer.update(fans_running_raw);
    _digital_inputs.fans_are_running.write(_parameters.di_fans_are_running_override.read()
                                               ? _parameters.di_fans_are_running_override_value.read()
                                               : _di_fans_are_running_debouncer.get_output());
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

void sfc_grid_side_external_ios_datamodel::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    static constexpr std::string_view override_component = "grid_io_override";
    static constexpr std::string_view config_component = "grid_io_configurations";

    // Digital input polarity configuration (ACTIVE_HIGH=0, ACTIVE_LOW=1)
    handler.register_parameter(override_component, "di_auxiliaries_are_on_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_auxiliaries_are_on_polarity));
    handler.register_parameter(override_component, "di_mv_breaker_is_closed_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_mv_breaker_is_closed_polarity));
    handler.register_parameter(override_component, "di_start_command_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_start_command_polarity));
    handler.register_parameter(override_component, "di_stop_command_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_stop_command_polarity));
    handler.register_parameter(override_component, "di_mv_breaker_is_tripping_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_mv_breaker_is_tripping_polarity));
    handler.register_parameter(override_component, "di_emergency_off_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_emergency_off_polarity));
    handler.register_parameter(override_component, "di_door_is_open_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_door_is_open_polarity));
    handler.register_parameter(override_component, "di_external_fault_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_external_fault_polarity));
    handler.register_parameter(override_component, "di_fans_are_running_polarity",
        yeet::utils::make_enum_accessor(_parameters.di_fans_are_running_polarity));

    // Digital input override parameters (override flag + override value pairs)
    handler.register_parameter(override_component, "di_auxiliaries_are_on_override",
        yeet::utils::make_bool_accessor(_parameters.di_auxiliaries_are_on_override));
    handler.register_parameter(override_component, "di_auxiliaries_are_on_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_auxiliaries_are_on_override_value));

    handler.register_parameter(override_component, "di_mv_breaker_is_closed_override",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_closed_override));
    handler.register_parameter(override_component, "di_mv_breaker_is_closed_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_closed_override_value));

    handler.register_parameter(override_component, "di_start_command_override",
        yeet::utils::make_bool_accessor(_parameters.di_start_command_override));
    handler.register_parameter(override_component, "di_start_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_start_command_override_value));

    handler.register_parameter(override_component, "di_stop_command_override",
        yeet::utils::make_bool_accessor(_parameters.di_stop_command_override));
    handler.register_parameter(override_component, "di_stop_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_stop_command_override_value));

    handler.register_parameter(override_component, "di_mv_breaker_is_tripping_override",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_tripping_override));
    handler.register_parameter(override_component, "di_mv_breaker_is_tripping_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_mv_breaker_is_tripping_override_value));

    handler.register_parameter(override_component, "di_emergency_off_override",
        yeet::utils::make_bool_accessor(_parameters.di_emergency_off_override));
    handler.register_parameter(override_component, "di_emergency_off_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_emergency_off_override_value));

    handler.register_parameter(override_component, "di_door_is_open_override",
        yeet::utils::make_bool_accessor(_parameters.di_door_is_open_override));
    handler.register_parameter(override_component, "di_door_is_open_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_door_is_open_override_value));

    handler.register_parameter(override_component, "di_external_fault_override",
        yeet::utils::make_bool_accessor(_parameters.di_external_fault_override));
    handler.register_parameter(override_component, "di_external_fault_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_external_fault_override_value));

    handler.register_parameter(override_component, "di_fans_are_running_override",
        yeet::utils::make_bool_accessor(_parameters.di_fans_are_running_override));
    handler.register_parameter(override_component, "di_fans_are_running_override_value",
        yeet::utils::make_bool_accessor(_parameters.di_fans_are_running_override_value));

    // Digital output override parameters (override flag + override value pairs)
    handler.register_parameter(override_component, "do_mv_breaker_close_override",
        yeet::utils::make_bool_accessor(_parameters.do_mv_breaker_close_override));
    handler.register_parameter(override_component, "do_mv_breaker_close_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_mv_breaker_close_override_value));

    handler.register_parameter(override_component, "do_mv_breaker_open_override",
        yeet::utils::make_bool_accessor(_parameters.do_mv_breaker_open_override));
    handler.register_parameter(override_component, "do_mv_breaker_open_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_mv_breaker_open_override_value));

    handler.register_parameter(override_component, "do_auxiliaries_on_command_override",
        yeet::utils::make_bool_accessor(_parameters.do_auxiliaries_on_command_override));
    handler.register_parameter(override_component, "do_auxiliaries_on_command_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_auxiliaries_on_command_override_value));

    handler.register_parameter(override_component, "do_on_command_acu_fan_1_override",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_1_override));
    handler.register_parameter(override_component, "do_on_command_acu_fan_1_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_1_override_value));

    handler.register_parameter(override_component, "do_on_command_acu_fan_2_override",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_2_override));
    handler.register_parameter(override_component, "do_on_command_acu_fan_2_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_2_override_value));

    handler.register_parameter(override_component, "do_on_command_acu_fan_3_override",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_3_override));
    handler.register_parameter(override_component, "do_on_command_acu_fan_3_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_3_override_value));

    handler.register_parameter(override_component, "do_on_command_acu_fan_4_override",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_4_override));
    handler.register_parameter(override_component, "do_on_command_acu_fan_4_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_on_command_acu_fan_4_override_value));

    handler.register_parameter(override_component, "do_software_trip_1_override",
        yeet::utils::make_bool_accessor(_parameters.do_software_trip_1_override));
    handler.register_parameter(override_component, "do_software_trip_1_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_software_trip_1_override_value));

    handler.register_parameter(override_component, "do_software_trip_2_override",
        yeet::utils::make_bool_accessor(_parameters.do_software_trip_2_override));
    handler.register_parameter(override_component, "do_software_trip_2_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_software_trip_2_override_value));

    handler.register_parameter(override_component, "do_software_trip_3_override",
        yeet::utils::make_bool_accessor(_parameters.do_software_trip_3_override));
    handler.register_parameter(override_component, "do_software_trip_3_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_software_trip_3_override_value));

    handler.register_parameter(override_component, "do_emergency_off_reset_override",
        yeet::utils::make_bool_accessor(_parameters.do_emergency_off_reset_override));
    handler.register_parameter(override_component, "do_emergency_off_reset_override_value",
        yeet::utils::make_bool_accessor(_parameters.do_emergency_off_reset_override_value));

    // ==================== IO Index Mapping Parameters ====================
    // Keep IO index mapping parameters in a separate component so they don't mix
    // with override parameters in UIs.

    // Digital input mapping indices
    handler.register_parameter(config_component, "di_auxiliaries_are_on_index",
        yeet::utils::make_units_accessor(_parameters.di_auxiliaries_are_on_index));
    handler.register_parameter(config_component, "di_mv_breaker_is_closed_index",
        yeet::utils::make_units_accessor(_parameters.di_mv_breaker_is_closed_index));
    handler.register_parameter(config_component, "di_start_command_index",
        yeet::utils::make_units_accessor(_parameters.di_start_command_index));
    handler.register_parameter(
        config_component, "di_stop_command_index", yeet::utils::make_units_accessor(_parameters.di_stop_command_index));
    handler.register_parameter(config_component, "di_mv_breaker_is_tripping_index",
        yeet::utils::make_units_accessor(_parameters.di_mv_breaker_is_tripping_index));
    handler.register_parameter(config_component, "di_emergency_off_index",
        yeet::utils::make_units_accessor(_parameters.di_emergency_off_index));
    handler.register_parameter(
        config_component, "di_door_is_open_index", yeet::utils::make_units_accessor(_parameters.di_door_is_open_index));
    handler.register_parameter(config_component, "di_external_fault_index",
        yeet::utils::make_units_accessor(_parameters.di_external_fault_index));
    handler.register_parameter(config_component, "di_fans_are_running_index",
        yeet::utils::make_units_accessor(_parameters.di_fans_are_running_index));

    // Digital output mapping indices
    handler.register_parameter(config_component, "do_mv_breaker_close_index",
        yeet::utils::make_units_accessor(_parameters.do_mv_breaker_close_index));
    handler.register_parameter(config_component, "do_mv_breaker_open_index",
        yeet::utils::make_units_accessor(_parameters.do_mv_breaker_open_index));
    handler.register_parameter(config_component, "do_auxiliaries_on_command_index",
        yeet::utils::make_units_accessor(_parameters.do_auxiliaries_on_command_index));
    handler.register_parameter(config_component, "do_on_command_acu_fan_1_index",
        yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_1_index));
    handler.register_parameter(config_component, "do_on_command_acu_fan_2_index",
        yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_2_index));
    handler.register_parameter(config_component, "do_on_command_acu_fan_3_index",
        yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_3_index));
    handler.register_parameter(config_component, "do_on_command_acu_fan_4_index",
        yeet::utils::make_units_accessor(_parameters.do_on_command_acu_fan_4_index));
    handler.register_parameter(config_component, "do_software_trip_1_index",
        yeet::utils::make_units_accessor(_parameters.do_software_trip_1_index));
    handler.register_parameter(config_component, "do_software_trip_2_index",
        yeet::utils::make_units_accessor(_parameters.do_software_trip_2_index));
    handler.register_parameter(config_component, "do_software_trip_3_index",
        yeet::utils::make_units_accessor(_parameters.do_software_trip_3_index));
    handler.register_parameter(config_component, "do_emergency_off_reset_index",
        yeet::utils::make_units_accessor(_parameters.do_emergency_off_reset_index));

    // Digital output polarity configuration
    handler.register_parameter(config_component, "do_mv_breaker_close_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_mv_breaker_close_polarity));
    handler.register_parameter(config_component, "do_mv_breaker_open_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_mv_breaker_open_polarity));
    handler.register_parameter(config_component, "do_auxiliaries_on_command_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_auxiliaries_on_command_polarity));
    handler.register_parameter(config_component, "do_on_command_acu_fan_1_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_1_polarity));
    handler.register_parameter(config_component, "do_on_command_acu_fan_2_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_2_polarity));
    handler.register_parameter(config_component, "do_on_command_acu_fan_3_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_3_polarity));
    handler.register_parameter(config_component, "do_on_command_acu_fan_4_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_on_command_acu_fan_4_polarity));
    handler.register_parameter(config_component, "do_software_trip_1_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_software_trip_1_polarity));
    handler.register_parameter(config_component, "do_software_trip_2_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_software_trip_2_polarity));
    handler.register_parameter(config_component, "do_software_trip_3_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_software_trip_3_polarity));
    handler.register_parameter(config_component, "do_emergency_off_reset_polarity",
        yeet::utils::make_enum_accessor(_parameters.do_emergency_off_reset_polarity));
}

bool sfc_grid_side_external_ios_datamodel::get_mv_breaker_close() const
{
    bool signal = _parameters.do_mv_breaker_close_override.read()
                      ? _parameters.do_mv_breaker_close_override_value.read()
                      : _digital_outputs.mv_breaker_close.read();
    // Apply polarity inversion for output
    return (_parameters.do_mv_breaker_close_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_mv_breaker_open() const
{
    bool signal = _parameters.do_mv_breaker_open_override.read() ? _parameters.do_mv_breaker_open_override_value.read()
                                                                 : _digital_outputs.mv_breaker_open.read();
    // Apply polarity inversion for output
    return (_parameters.do_mv_breaker_open_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_auxiliaries_on_command() const
{
    bool signal = _parameters.do_auxiliaries_on_command_override.read()
                      ? _parameters.do_auxiliaries_on_command_override_value.read()
                      : _digital_outputs.auxiliaries_on_command.read();
    // Apply polarity inversion for output
    return (_parameters.do_auxiliaries_on_command_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

// ACU fan ON command getters (all apply polarity inversion)
bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_1() const
{
    bool signal = _parameters.do_on_command_acu_fan_1_override.read()
                      ? _parameters.do_on_command_acu_fan_1_override_value.read()
                      : _digital_outputs.on_command_acu_fan_1.read();
    return (_parameters.do_on_command_acu_fan_1_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_2() const
{
    bool signal = _parameters.do_on_command_acu_fan_2_override.read()
                      ? _parameters.do_on_command_acu_fan_2_override_value.read()
                      : _digital_outputs.on_command_acu_fan_2.read();
    return (_parameters.do_on_command_acu_fan_2_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_3() const
{
    bool signal = _parameters.do_on_command_acu_fan_3_override.read()
                      ? _parameters.do_on_command_acu_fan_3_override_value.read()
                      : _digital_outputs.on_command_acu_fan_3.read();
    return (_parameters.do_on_command_acu_fan_3_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_on_command_acu_fan_4() const
{
    bool signal = _parameters.do_on_command_acu_fan_4_override.read()
                      ? _parameters.do_on_command_acu_fan_4_override_value.read()
                      : _digital_outputs.on_command_acu_fan_4.read();
    return (_parameters.do_on_command_acu_fan_4_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

// Software trip signal getters (all apply polarity inversion)
bool sfc_grid_side_external_ios_datamodel::get_software_trip_1() const
{
    bool signal = _parameters.do_software_trip_1_override.read() ? _parameters.do_software_trip_1_override_value.read()
                                                                 : _digital_outputs.software_trip_1.read();
    return (_parameters.do_software_trip_1_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_software_trip_2() const
{
    bool signal = _parameters.do_software_trip_2_override.read() ? _parameters.do_software_trip_2_override_value.read()
                                                                 : _digital_outputs.software_trip_2.read();
    return (_parameters.do_software_trip_2_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_software_trip_3() const
{
    bool signal = _parameters.do_software_trip_3_override.read() ? _parameters.do_software_trip_3_override_value.read()
                                                                 : _digital_outputs.software_trip_3.read();
    return (_parameters.do_software_trip_3_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

bool sfc_grid_side_external_ios_datamodel::get_emergency_off_reset() const
{
    bool signal = _parameters.do_emergency_off_reset_override.read()
                      ? _parameters.do_emergency_off_reset_override_value.read()
                      : _digital_outputs.emergency_off_reset.read();
    return (_parameters.do_emergency_off_reset_polarity.read() == io_polarity_t::ACTIVE_LOW) ? !signal : signal;
}

}  // namespace yeet::application::sfc::integration
