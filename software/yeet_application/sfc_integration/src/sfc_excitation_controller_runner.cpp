#include "sfc_integration/sfc_excitation_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_excitation_controller_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_excitation_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_excitation_controller_runner::initialize()
{
    _sfc_excitation_controller.reset();
    set_parameters();
    set_parameter_callback();
}

void sfc_excitation_controller_runner::run()
{
    // Update the excitation controller with enable command from startup sequencer
    const bool enable_excitation_command = _inputs.release_excitation.read();
    _sfc_excitation_controller.update(enable_excitation_command);

    // Determine field current reference (apply override if enabled)
    current_pu_t field_current_reference;
    if (_parameters.override_field_current_reference.read()) {
        field_current_reference = _parameters.field_current_reference_override_value.read();
    }
    else {
        field_current_reference = _sfc_excitation_controller.get_field_current_reference_pu();
    }

    // Write outputs
    _outputs.field_current_reference.write(field_current_reference);
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
    // Configure constant field current setpoint and flux build-up duration
    _sfc_excitation_controller.configure_field_current_setpoint(_parameters.field_current_setpoint.read());
    _sfc_excitation_controller.configure_flux_buildup_duration(
        _parameters.prefluxing_duration.read());  // Renamed method

    // Commented out - simplified version doesn't use speed threshold
    // _sfc_excitation_controller.configure_constant_flux_speed_threshold(
    //     _parameters.constant_flux_speed_threshold.read());
}

void sfc_excitation_controller_runner::set_parameter_callback()
{
    _parameters.field_current_setpoint.register_callback([&]() { set_parameters(); });
    _parameters.prefluxing_duration.register_callback([&]() { set_parameters(); });

    // Commented out - simplified version doesn't use speed threshold parameter
    // _parameters.constant_flux_speed_threshold.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
