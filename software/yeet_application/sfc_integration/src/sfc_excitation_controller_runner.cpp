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

    // Initialize edge detection flags
    _prev_start_command = false;
    _prev_stop_command = false;
}

void sfc_excitation_controller_runner::run()
{
    // Handle start/stop commands with edge detection
    handle_start_stop_commands();

    // Update the excitation controller
    _sfc_excitation_controller.update(_inputs.machine_rotor_speed_pu.read(), _inputs.exciter_ready.read());

    // Write outputs
    _outputs.excitation_voltage_setpoint_pu.write(_sfc_excitation_controller.get_excitation_voltage_setpoint_pu());
    _outputs.enable_excitation.write(_sfc_excitation_controller.get_enable_excitation());
    _outputs.is_field_established.write(_sfc_excitation_controller.get_is_field_established());
    _outputs.is_running.write(_sfc_excitation_controller.get_is_running());
    _outputs.prefluxing_elapsed_time.write(_sfc_excitation_controller.get_prefluxing_elapsed_time());
}

void sfc_excitation_controller_runner::stop()
{
    _sfc_excitation_controller.stop_excitation();
    _sfc_excitation_controller.reset();
}

void sfc_excitation_controller_runner::set_parameters()
{
    // Configure timing parameters
    _sfc_excitation_controller.configure_prefluxing_duration(_parameters.prefluxing_duration.read());
    _sfc_excitation_controller.configure_field_established_timeout(_parameters.field_established_timeout.read());
    _sfc_excitation_controller.configure_excitation_ramp_down_time(_parameters.excitation_ramp_down_time.read());

    // Configure lookup table
    _sfc_excitation_controller.configure_excitation_voltage_lookup_table(
        _parameters.speed_axis_pu.read(), _parameters.excitation_voltage_axis_pu.read());
}

void sfc_excitation_controller_runner::set_parameter_callback()
{
    _parameters.prefluxing_duration.register_callback([&]() { set_parameters(); });
    _parameters.field_established_timeout.register_callback([&]() { set_parameters(); });
    _parameters.excitation_ramp_down_time.register_callback([&]() { set_parameters(); });
    _parameters.speed_axis_pu.register_callback([&]() { set_parameters(); });
    _parameters.excitation_voltage_axis_pu.register_callback([&]() { set_parameters(); });
}

void sfc_excitation_controller_runner::handle_start_stop_commands()
{
    // Read current command states
    const bool current_start_command = _inputs.start_command.read();
    const bool current_stop_command = _inputs.stop_command.read();

    // Detect rising edge on start command
    if (current_start_command && !_prev_start_command) {
        _sfc_excitation_controller.start_excitation();
    }

    // Detect rising edge on stop command
    if (current_stop_command && !_prev_stop_command) {
        _sfc_excitation_controller.stop_excitation();
    }

    // Update previous states for next cycle
    _prev_start_command = current_start_command;
    _prev_stop_command = current_stop_command;
}

}  // namespace yeet::application::sfc::integration
