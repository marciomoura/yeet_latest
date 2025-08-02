#include "sfc_integration/sfc_current_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_current_controller_runner::initialize() {
    set_parameters();
    set_parameter_callback();
    _sfc_current_controller.reset();
}

void sfc_current_controller_runner::run() {
    _sfc_current_controller.update(_inputs.current_reference_pu, _inputs.dc_link_current_measured_pu,
                                   _inputs.ideal_dc_voltage_pu, _inputs.inverter_back_emf_machine_pu);

    _outputs.firing_angle_reference = _sfc_current_controller.get_firing_angle_reference_radians();

    // Calculate current error
    current_pu_t current_ref = _inputs.current_reference_pu;
    current_pu_t current_meas = _inputs.dc_link_current_measured_pu;
    _outputs.current_error = current_pu_t(current_ref.value() - current_meas.value());

    _outputs.pi_output = _sfc_current_controller.get_firing_angle_reference_radians();
    _outputs.is_saturated = false;  // This should be implemented in the controller
    _outputs.is_active = true;      // Assume always active for now
}

void sfc_current_controller_runner::stop() { _sfc_current_controller.reset(); }

void sfc_current_controller_runner::set_parameters() {
    // Convert integral gain to integral time constant
    duration_t integral_time = duration_t(1.0 / _parameters.pi_integral_gain);

    _sfc_current_controller.configure_pi_gains(_parameters.pi_proportional_gain, integral_time);
    _sfc_current_controller.configure_firing_angle_limits_radians(_parameters.min_firing_angle,
                                                                  _parameters.max_firing_angle);

    current_pu_t current_limit = _parameters.current_reference_magnitude_limit;
    _sfc_current_controller.configure_current_limits_pu(current_pu_t(-current_limit.value()), current_limit);
    _sfc_current_controller.configure_current_reference_slew_rate(_parameters.current_reference_rate_limit);
}

void sfc_current_controller_runner::set_parameter_callback() {
    _parameters.pi_proportional_gain.register_callback([&]() { set_parameters(); });
    _parameters.pi_integral_gain.register_callback([&]() { set_parameters(); });
    _parameters.min_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.max_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.current_reference_rate_limit.register_callback([&]() { set_parameters(); });
    _parameters.current_reference_magnitude_limit.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration