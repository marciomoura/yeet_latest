#include "sfc/sfc_current_controller.hpp"

#include <algorithm>
#include <cmath>

namespace yeet::application::sfc {

sfc_current_controller::sfc_current_controller(double sampling_time)
    : _pi_controller(sampling_time), _firing_angle_limiter(2.0f, sampling_time, 0.0f) {
    // The PI output is a per-unit voltage. A reasonable limit for the DC voltage is ~1.5 pu.
    _pi_controller.set_output_limits(-1.5, 1.5);
    _pi_controller.enable_integrator_clamping(true);
    // _firing_angle_limiter.configure(0.2f);
}

void sfc_current_controller::set_pi_gains(double kp, double ti) { _pi_controller.configure_with_ti(kp, ti); }

void sfc_current_controller::set_current_limits_pu(double min_pu, double max_pu) {
    _dc_link_current_min_pu = min_pu;
    _dc_link_current_max_pu = max_pu;
}

void sfc_current_controller::set_firing_angle_limits_radians(double min_radians, double max_radians) {
    _grid_firing_angle_min_radians = min_radians;
    _grid_firing_angle_max_radians = max_radians;
}

double sfc_current_controller::update(double dc_link_current_ref_pu, double dc_link_current_measured_pu,
                                      double dc_link_voltage_inverter_estimated_pu) {
    // Step 1: Limit the incoming current reference and add a rate of change limiter
    // This is to prevent sudden changes in the firing angle reference.
    _firing_angle_limiter.update(dc_link_current_ref_pu);

    _current_ref_limited_pu =
        std::clamp(_firing_angle_limiter.get_output(), _dc_link_current_min_pu, _dc_link_current_max_pu);

    // Step 2: Calculate the error signal
    const double current_error_pu = _current_ref_limited_pu - dc_link_current_measured_pu;

    // Step 3: Update the PI controller
    const double pi_output_voltage_pu = _pi_controller.update(current_error_pu);

    // Step 4: Add feedforward compensation. The result is assumed to be cos(alpha).
    const double cos_alpha = pi_output_voltage_pu + dc_link_voltage_inverter_estimated_pu;

    // Step 5: Clamp the result and compute the final angle
    _cos_alpha_clamped = std::clamp(cos_alpha, -1.0, 1.0);
    const double grid_firing_angle_ref_unclamped_radians = std::acos(_cos_alpha_clamped);

    // Step 6: Clamp the final firing angle to the configured limits and store it
    _firing_angle_ref_radians = std::clamp(grid_firing_angle_ref_unclamped_radians, _grid_firing_angle_min_radians,
                                           _grid_firing_angle_max_radians);

    return _firing_angle_ref_radians;
}

double sfc_current_controller::get_firing_angle_ref_radians() const { return _firing_angle_ref_radians; }

double sfc_current_controller::get_current_ref_limited_pu() const { return _current_ref_limited_pu; }

double sfc_current_controller::get_cos_alpha_clamped() const { return _cos_alpha_clamped; }

}  // namespace yeet::application::sfc