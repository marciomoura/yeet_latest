#include "sfc_integration/sfc_excitation_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_excitation_controller_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_excitation_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_excitation_controller_runner::initialize()
{
    _sfc_excitation_controller.reset();
    set_parameters();
    set_parameter_callback();
}

void sfc_excitation_controller_runner::run()
{
    // Update the excitation controller with enable command from startup sequencer (or force parameter)
    const bool enable_excitation_command =
        _inputs.release_excitation.read() || _parameters.force_release_excitation.read();

    // Update the excitation controller with enable command from startup sequencer
    const bool enable_excitation_command = _inputs.release_excitation.read();
    const auto machine_speed_pu = _inputs.machine_rotor_speed_pu.read();
    const auto machine_voltage_magnitude_pu = _inputs.machine_voltage_magnitude_pu.read();
    _sfc_excitation_controller.update(machine_speed_pu, machine_voltage_magnitude_pu, enable_excitation_command);

    // Determine field current reference (apply override if enabled)
    current_pu_t field_current_reference{};
    if (_parameters.field_current_reference_override.read()) {
        _sfc_excitation_controller.reset();
        field_current_reference = _parameters.field_current_reference_override_value.read();
    }
    else {
        field_current_reference = _sfc_excitation_controller.get_field_current_reference_pu();
    }

    // Write outputs
    _outputs.field_current_reference.write(field_current_reference);

    _outputs.machine_voltage_reference_pu.write(_sfc_excitation_controller.get_machine_voltage_reference_pu());
    _outputs.machine_voltage_magnitude_pu.write(machine_voltage_magnitude_pu);
    _outputs.machine_voltage_error_pu.write(_sfc_excitation_controller.get_machine_voltage_error_pu());

    _outputs.prefluxing_elapsed_time.write(
        _sfc_excitation_controller.get_flux_buildup_elapsed_time());  // Renamed getter
    _outputs.is_field_established.write(_sfc_excitation_controller.get_is_field_established());

    // Write command outputs
    const bool enable_excitation = _sfc_excitation_controller.get_enable_excitation();
    _outputs.excitation_start_command.write(enable_excitation);
    _outputs.excitation_stop_command.write(!enable_excitation);
}

void sfc_excitation_controller_runner::stop() { _sfc_excitation_controller.reset(); }

void sfc_excitation_controller_runner::set_parameters()
{
    // Configure forced current and release thresholds
    _sfc_excitation_controller.configure_forced_field_current(_parameters.forced_field_current_pu.read());
    _sfc_excitation_controller.configure_release_voltage_threshold(_parameters.release_voltage_threshold_pu.read());
    _sfc_excitation_controller.configure_release_speed_threshold(_parameters.release_speed_threshold_pu.read());
    _sfc_excitation_controller.configure_flux_buildup_duration(_parameters.prefluxing_duration.read());

    // Configure voltage control PI loop
    _sfc_excitation_controller.configure_voltage_reference_lookup_table(
        _parameters.voltage_reference_speed_axis_pu.read(), _parameters.voltage_reference_pu.read());
    _sfc_excitation_controller.configure_voltage_pi_gains(
        _parameters.voltage_pi_kp.read(), _parameters.voltage_pi_ti.read());
    _sfc_excitation_controller.configure_voltage_pi_output_limits(
        _parameters.voltage_pi_output_min_pu.read(), _parameters.voltage_pi_output_max_pu.read());
}

void sfc_excitation_controller_runner::set_parameter_callback()
{
    _parameters.forced_field_current_pu.register_callback([&]() {
        _sfc_excitation_controller.configure_forced_field_current(_parameters.forced_field_current_pu.read());
    });

    _parameters.release_voltage_threshold_pu.register_callback([&]() {
        _sfc_excitation_controller.configure_release_voltage_threshold(_parameters.release_voltage_threshold_pu.read());
    });

    _parameters.release_speed_threshold_pu.register_callback([&]() {
        _sfc_excitation_controller.configure_release_speed_threshold(_parameters.release_speed_threshold_pu.read());
    });

    _parameters.voltage_reference_speed_axis_pu.register_callback([&]() {
        _sfc_excitation_controller.configure_voltage_reference_lookup_table(
            _parameters.voltage_reference_speed_axis_pu.read(), _parameters.voltage_reference_pu.read());
    });

    _parameters.voltage_reference_pu.register_callback([&]() {
        _sfc_excitation_controller.configure_voltage_reference_lookup_table(
            _parameters.voltage_reference_speed_axis_pu.read(), _parameters.voltage_reference_pu.read());
    });

    _parameters.voltage_pi_kp.register_callback([&]() {
        _sfc_excitation_controller.configure_voltage_pi_gains(
            _parameters.voltage_pi_kp.read(), _parameters.voltage_pi_ti.read());
    });

    _parameters.voltage_pi_ti.register_callback([&]() {
        _sfc_excitation_controller.configure_voltage_pi_gains(
            _parameters.voltage_pi_kp.read(), _parameters.voltage_pi_ti.read());
    });

    _parameters.voltage_pi_output_min_pu.register_callback([&]() {
        _sfc_excitation_controller.configure_voltage_pi_output_limits(
            _parameters.voltage_pi_output_min_pu.read(), _parameters.voltage_pi_output_max_pu.read());
    });

    _parameters.voltage_pi_output_max_pu.register_callback([&]() {
        _sfc_excitation_controller.configure_voltage_pi_output_limits(
            _parameters.voltage_pi_output_min_pu.read(), _parameters.voltage_pi_output_max_pu.read());
    });

    _parameters.prefluxing_duration.register_callback(
        [&]() { _sfc_excitation_controller.configure_flux_buildup_duration(_parameters.prefluxing_duration.read()); });
}

}  // namespace yeet::application::sfc::integration
