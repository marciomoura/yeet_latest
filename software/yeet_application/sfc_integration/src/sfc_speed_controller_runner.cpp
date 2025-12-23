#include "sfc_integration/sfc_speed_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_speed_controller_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_speed_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_speed_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    _sfc_speed_controller.reset(_parameters.initial_speed);
}

void sfc_speed_controller_runner::run()
{
    const bool current_release_state = _inputs.release_pulses.read();
    const bool current_preset_request = _inputs.preset_request.read();
    const bool release_rising_edge = _release_edge_detector.update(current_release_state);

    // Bumpless transfer: supervisor provides a one-shot preset_request at handover (signal level).
    // Keep a release-edge fallback for safety in case preset_request is not wired.
    const bool preset = current_preset_request || release_rising_edge;

    if (preset) {
        _sfc_speed_controller.preset_for_bumpless_transfer(
            _inputs.applied_torque_reference, _inputs.speed_reference, _inputs.measured_speed);
    }

    // Execute controller update
    if (current_release_state) {
        // Combine: freeze if downstream is saturated OR supervisor requests freeze
        const bool freeze_integrator = _inputs.is_saturated || _inputs.freeze_integrator;

        _sfc_speed_controller.freeze_integrator(freeze_integrator);
        _sfc_speed_controller.update(_inputs.speed_reference, _inputs.measured_speed);

        // Write output signals
        _outputs.torque_reference = _sfc_speed_controller.get_torque_ref_pu();
        _outputs.pi_output = _sfc_speed_controller.get_pi_output();
        _outputs.is_saturated = _sfc_speed_controller.is_saturated();
    }
    else {
        // Write zero outputs when not enabled
        _outputs.torque_reference = torque_pu_t{0.0};
        _outputs.pi_output = real_t{0.0};
        _outputs.is_saturated = false;
    }
}

void sfc_speed_controller_runner::stop() { _sfc_speed_controller.reset(); }

void sfc_speed_controller_runner::set_parameters()
{
    _sfc_speed_controller.configure_pi_gains(_parameters.kp, _parameters.ti);
    _sfc_speed_controller.configure_torque_limits_pu(
        -_parameters.torque_reference_magnitude_limit.read(), _parameters.torque_reference_magnitude_limit.read());
    _sfc_speed_controller.configure_feedforward_pu(_parameters.feedforward_inertia);
}

void sfc_speed_controller_runner::set_parameter_callback()
{
    _parameters.kp.register_callback(
        [&]() { _sfc_speed_controller.configure_pi_gains(_parameters.kp, _parameters.ti); });
    _parameters.ti.register_callback(
        [&]() { _sfc_speed_controller.configure_pi_gains(_parameters.kp, _parameters.ti); });
    _parameters.torque_reference_magnitude_limit.register_callback([&]() {
        _sfc_speed_controller.configure_torque_limits_pu(
            -_parameters.torque_reference_magnitude_limit.read(), _parameters.torque_reference_magnitude_limit.read());
    });
    _parameters.feedforward_inertia.register_callback(
        [&]() { _sfc_speed_controller.configure_feedforward_pu(_parameters.feedforward_inertia); });
}

}  // namespace yeet::application::sfc::integration