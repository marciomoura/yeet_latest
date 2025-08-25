#include "sfc_integration/sfc_current_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_current_controller_runner::initialize()
{
    set_controller_parameters();
    set_limit_parameters();
    set_parameter_callback();

    _sfc_current_controller.enable_current_reference_slew_rate(false);
    _sfc_current_controller.reset();
    // _sfc_current_controller.configure_pi_gains(0.5, duration_t(ms(100.0)));

    // Desired Crossover Frequency: 20.0 Hz
    // Desired Phase Margin: 60.0 deg
    // Actual Gain Margin: 12.01 dB @ 20.00 Hz
    // Actual Phase Margin: 60.00 deg @ 63.09 Hz
    // Processing: f_c=20.0Hz, PM=60.0°, f_filt=100.0Hz
    //   Kp=0.7439, Ti=101.64ms
    // _sfc_current_controller.configure_pi_gains(0.7439 / 1.35, duration_t(ms(101.64)));
    _sfc_current_controller.configure_pi_gains(0.7439 / 1.35, duration_t(ms(101.64)));
}

void sfc_current_controller_runner::run()
{
    _sfc_current_controller.freeze_integrator(_inputs.freeze_integrator.read());

    _sfc_current_controller.update(_inputs.dc_link_current_reference.read(), _inputs.dc_link_current_measured.read(),
        _inputs.grid_side_voltage_magnitude.read(), _inputs.machine_side_voltage_magnitude.read());

    _outputs.firing_angle.write(
        angle_wrapped::from_radians(_sfc_current_controller.get_firing_angle_reference_radians()));

    _outputs.current_error.write(_inputs.dc_link_current_reference.read() - _inputs.dc_link_current_measured.read());
    _outputs.current_controller_is_saturated.write(_sfc_current_controller.is_saturated());
}

void sfc_current_controller_runner::stop() { _sfc_current_controller.reset(); }

void sfc_current_controller_runner::set_limit_parameters()
{
    const angle_t min_firing_angle = _parameters.min_firing_angle.read().get_radians();
    const angle_t max_firing_angle = _parameters.max_firing_angle.read().get_radians();
    const real_t current_reference_rate_limit = _parameters.current_reference_rate_limit.read();
    const current_pu_t current_reference_magnitude_limit = _parameters.current_reference_magnitude_limit.read();

    _sfc_current_controller.configure_firing_angle_limits_radians(min_firing_angle, max_firing_angle);
    _sfc_current_controller.configure_current_limits_pu(
        -current_reference_magnitude_limit, current_reference_magnitude_limit);
    _sfc_current_controller.configure_current_reference_slew_rate(current_reference_rate_limit);
}

void sfc_current_controller_runner::set_controller_parameters()
{
    const real_t kp = _parameters.kp.read();
    const duration_t ti = _parameters.ti.read();
    // const real_t anti_windup_gain = _parameters.anti_windup_gain.read();

    _sfc_current_controller.configure_pi_gains(kp, ti);
}

void sfc_current_controller_runner::set_parameter_callback()
{
    _parameters.kp.register_callback([&]() { set_controller_parameters(); });
    _parameters.ti.register_callback([&]() { set_controller_parameters(); });
    // _parameters.anti_windup_gain.register_callback([&]() { set_controller_parameters(); });
    _parameters.min_firing_angle.register_callback([&]() { set_limit_parameters(); });
    _parameters.max_firing_angle.register_callback([&]() { set_limit_parameters(); });
    _parameters.current_reference_rate_limit.register_callback([&]() { set_limit_parameters(); });
    _parameters.current_reference_magnitude_limit.register_callback([&]() { set_limit_parameters(); });
}

}  // namespace yeet::application::sfc::integration