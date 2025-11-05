#include "sfc_integration/sfc_machine_side_external_ios_datamodel.hpp"

#include <cmath>

#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

sfc_machine_side_external_ios_datamodel::sfc_machine_side_external_ios_datamodel() { register_callbacks(); }

void sfc_machine_side_external_ios_datamodel::register_callbacks()
{
    auto excitation_inputs_callback = [this]() { write_excitation_overrides(); };

    auto speed_generator_profiles_callback = [this]() { write_speed_generator_overrides(); };

    _parameters.override_excitation_inputs.register_callback(excitation_inputs_callback);
    _parameters.exciter_ready.register_callback(excitation_inputs_callback);
    _parameters.excitation_start_command.register_callback(excitation_inputs_callback);
    _parameters.excitation_stop_command.register_callback(excitation_inputs_callback);

    _parameters.run_creep_speed.register_callback(excitation_inputs_callback);
    _parameters.run_purge_accel.register_callback(excitation_inputs_callback);
    _parameters.run_ignition_accel.register_callback(excitation_inputs_callback);
    _parameters.run_assist_accel.register_callback(excitation_inputs_callback);
    _parameters.hold_speed.register_callback(excitation_inputs_callback);
    _parameters.decelerate.register_callback(excitation_inputs_callback);
    _parameters.idle.register_callback(excitation_inputs_callback);
}

void sfc_machine_side_external_ios_datamodel::update_analog_inputs(
    yeet::control::adc_counts v_ab_counts, yeet::control::adc_counts v_bc_counts)
{
    _analog_inputs.v_ab_counts.write(v_ab_counts);
    _analog_inputs.v_bc_counts.write(v_bc_counts);
}

void sfc_machine_side_external_ios_datamodel::update_fiber_optics_inputs(sfc_operating_mode_t operating_mode,
    yeet::application::sfc::sfc_state_id current_state_id,
    bool force_commutation_acknowledged,
    bool current_is_zero,
    bool is_saturated,
    bool pulse_releases,
    current_pu_t dc_link_current,
    bool preset_request)
{
    _fiber_optics_inputs.operating_mode.write(operating_mode);
    _fiber_optics_inputs.current_state_id.write(current_state_id);
    _fiber_optics_inputs.force_commutation_acknowledged.write(force_commutation_acknowledged);
    _fiber_optics_inputs.current_is_zero.write(current_is_zero);
    _fiber_optics_inputs.is_saturated.write(is_saturated);
    _fiber_optics_inputs.pulse_releases.write(pulse_releases);
    _fiber_optics_inputs.dc_link_current.write(dc_link_current);
    _fiber_optics_inputs.preset_request.write(preset_request);
}

control::six_pulse_generator::pulses sfc_machine_side_external_ios_datamodel::get_thyristor_pulse_outputs() const
{
    return _thyristor_output.firing_pulses.read();
}

void sfc_machine_side_external_ios_datamodel::update_excitation_inputs(
    bool exciter_ready, bool excitation_start_command, bool excitation_stop_command)
{
    const bool excitation_overrides_written = write_excitation_overrides();
    if (!excitation_overrides_written) {
        _excitation_outputs.exciter_ready.write(exciter_ready);
        _excitation_outputs.excitation_start_command.write(excitation_start_command);
        _excitation_outputs.excitation_stop_command.write(excitation_stop_command);
    }
}

void sfc_machine_side_external_ios_datamodel::update_speed_generator_inputs(bool run_creep_speed,
    bool run_purge_accel,
    bool run_ignition_accel,
    bool run_assist_accel,
    bool hold_speed,
    bool decelerate,
    bool idle)
{
    const bool speed_generator_overrides_written = write_speed_generator_overrides();
    if (!speed_generator_overrides_written) {
        _speed_generator_outputs.run_creep_speed.write(run_creep_speed);
        _speed_generator_outputs.run_purge_accel.write(run_purge_accel);
        _speed_generator_outputs.run_ignition_accel.write(run_ignition_accel);
        _speed_generator_outputs.run_assist_accel.write(run_assist_accel);
        _speed_generator_outputs.hold_speed.write(hold_speed);
        _speed_generator_outputs.decelerate.write(decelerate);
        _speed_generator_outputs.idle.write(idle);
    }
}

bool sfc_machine_side_external_ios_datamodel::write_excitation_overrides()
{
    if (!_parameters.override_excitation_inputs.read()) {
        return false;
    }

    _excitation_outputs.exciter_ready.write(_parameters.exciter_ready.read());
    _excitation_outputs.excitation_start_command.write(_parameters.excitation_start_command.read());
    _excitation_outputs.excitation_stop_command.write(_parameters.excitation_stop_command.read());
    return true;
}

bool sfc_machine_side_external_ios_datamodel::write_speed_generator_overrides()
{
    if (!_parameters.override_speed_generator_profiles.read()) {
        return false;
    }

    _speed_generator_outputs.run_creep_speed.write(_parameters.run_creep_speed.read());
    _speed_generator_outputs.run_purge_accel.write(_parameters.run_purge_accel.read());
    _speed_generator_outputs.run_ignition_accel.write(_parameters.run_ignition_accel.read());
    _speed_generator_outputs.run_assist_accel.write(_parameters.run_assist_accel.read());
    _speed_generator_outputs.hold_speed.write(_parameters.hold_speed.read());
    _speed_generator_outputs.decelerate.write(_parameters.decelerate.read());
    _speed_generator_outputs.idle.write(_parameters.idle.read());
    return true;
}

void sfc_machine_side_external_ios_datamodel::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Register grid-side base quantities
    entry.parameters.insert(
        "exc_override_excitation_inputs", yeet::utils::make_units_accessor(_parameters.override_excitation_inputs));
    entry.parameters.insert("exc_exciter_ready", yeet::utils::make_units_accessor(_parameters.exciter_ready));
    entry.parameters.insert(
        "exc_excitation_start_command", yeet::utils::make_units_accessor(_parameters.excitation_start_command));
    entry.parameters.insert(
        "exc_excitation_stop_command", yeet::utils::make_units_accessor(_parameters.excitation_stop_command));

    entry.parameters.insert("speed_ref_override_speed_generator_profiles",
        yeet::utils::make_units_accessor(_parameters.override_speed_generator_profiles));
    entry.parameters.insert("speed_ref_run_creep_speed", yeet::utils::make_units_accessor(_parameters.run_creep_speed));
    entry.parameters.insert("speed_ref_run_purge_accel", yeet::utils::make_units_accessor(_parameters.run_purge_accel));
    entry.parameters.insert(
        "speed_ref_run_ignition_accel", yeet::utils::make_units_accessor(_parameters.run_ignition_accel));
    entry.parameters.insert(
        "speed_ref_run_assist_accel", yeet::utils::make_units_accessor(_parameters.run_assist_accel));
    entry.parameters.insert("speed_ref_hold_speed", yeet::utils::make_units_accessor(_parameters.hold_speed));
    entry.parameters.insert("speed_ref_decelerate", yeet::utils::make_units_accessor(_parameters.decelerate));
    entry.parameters.insert("speed_ref_idle", yeet::utils::make_units_accessor(_parameters.idle));

    // Register the entry with the model registry
    registry.insert("machine_side_external_ios", std::move(entry));
}

}  // namespace yeet::application::sfc::integration
