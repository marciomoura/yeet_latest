#include "sfc/sfc_current_controller.hpp"

#include <algorithm>
#include <cmath>

#include "sfc/sfc_per_unit_system.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_current_controller::sfc_current_controller(duration_t sampling_time)
    : _pi_controller(sampling_time.value()),
      _rate_of_change_limiter(2.0, sampling_time.value(), current_pu_t{0.0}.value()) {
    // The PI output is a per-unit voltage. A reasonable limit for the DC voltage is ~1.5 pu.
    configure_pi_gains(0.2, duration_t(ms(100.0)));
    configure_current_limits_pu(current_pu_t(-1.5), current_pu_t(1.5));
    configure_firing_angle_limits_radians(angle_t(deg_to_rad(0.0)), angle_t(deg_to_rad(90.0)));
    recompute_internal_limits();
    _pi_controller.enable_integrator_clamping(true);
    _rate_of_change_limiter.configure_enable(false);  // By default, the rate limiter is disabled
}

void sfc_current_controller::configure_pi_gains(double proportional_gain, duration_t integral_time) {
    _pi_controller.configure_with_ti(proportional_gain, integral_time.value());
}

void sfc_current_controller::configure_current_limits_pu(current_pu_t min_current_pu, current_pu_t max_current_pu) {
    _dc_link_current_min_pu = min_current_pu;
    _dc_link_current_max_pu = max_current_pu;
}

void sfc_current_controller::configure_current_reference_slew_rate(double slew_rate_pu_per_second) {
    _rate_of_change_limiter.configure(slew_rate_pu_per_second);
}

void sfc_current_controller::enable_current_reference_slew_rate(bool enable) {
    _rate_of_change_limiter.configure_enable(enable);
}

void sfc_current_controller::configure_firing_angle_limits_radians(angle_t min_angle, angle_t max_angle) {
    _grid_firing_angle_min_radians = min_angle;
    _grid_firing_angle_max_radians = max_angle;
    recompute_internal_limits();
}

void sfc_current_controller::reset() {
    _pi_controller.reset();
    _rate_of_change_limiter.reset();
    _cos_alpha_clamped = 0.0;
    _firing_angle_reference_radians = angle_t{deg_to_rad(90.0)};  // Reset to 90 degrees
    _current_reference_limited_pu = current_pu_t{0.0};
}

void sfc_current_controller::update(current_pu_t dc_link_current_reference_pu, current_pu_t dc_link_current_measured_pu,
                                    voltage_pu_t ideal_dc_voltage_pu, voltage_pu_t inverter_back_emf_pu) {
    // Step 1: Saturate the incoming current reference to its absolute limits
    const auto saturated_current_reference_pu =
        std::clamp(dc_link_current_reference_pu, _dc_link_current_min_pu, _dc_link_current_max_pu);

    // Step 2: Apply a rate-of-change limit to the saturated reference for smooth transitions
    _rate_of_change_limiter.update(saturated_current_reference_pu.value());
    _current_reference_limited_pu = current_pu_t{_rate_of_change_limiter.get_output()};

    // Step 3: Calculate the error signal using the slew-rate-limited reference
    const auto current_error_pu = _current_reference_limited_pu - dc_link_current_measured_pu;

    // Step 4: Update the PI controller to get the corrective control voltage
    const auto v_ctrl_pi = _pi_controller.update(current_error_pu.value());

    // Step 5: Add feedforward compensation (V_ctrl_total = V_ctrl_PI + V_di_est)
    const auto v_ctrl_total = v_ctrl_pi + inverter_back_emf_pu.value();

    // Step 6: Linearize by normalizing with V_do to get cos(alpha) command
    // Avoid division by zero or small numbers for stability.
    const double v_do_safe = std::max(ideal_dc_voltage_pu.value(), 0.2);
    const double cos_alpha_cmd = v_ctrl_total / v_do_safe;

    // Step 7: Clamp the cosine of the firing angle based on pre-calculated limits
    _cos_alpha_clamped = std::clamp(cos_alpha_cmd, _cos_alpha_min, _cos_alpha_max);

    // Step 8: Compute the final angle and store it
    _firing_angle_reference_radians = angle_t{std::acos(_cos_alpha_clamped)};
}

void sfc_current_controller::update(current_pu_t dc_link_current_reference_pu, current_pu_t dc_link_current_measured_pu,
                                    voltage_pu_t ideal_dc_voltage_pu, voltage_pu_t inverter_back_emf_machine_pu,
                                    const voltage_t& machine_side_base_voltage,
                                    const precomputed_divisor<voltage_t>& grid_side_base_voltage) {
    // Reflect the machine-side inverter back-EMF to grid-side per-unit
    const voltage_pu_t inverter_back_emf_grid_reflected_pu = reflect_machine_to_grid_voltage_pu(
        inverter_back_emf_machine_pu, machine_side_base_voltage, grid_side_base_voltage);

    // Call the main update method with the reflected voltage
    update(dc_link_current_reference_pu, dc_link_current_measured_pu, ideal_dc_voltage_pu,
           inverter_back_emf_grid_reflected_pu);
}

angle_t sfc_current_controller::get_firing_angle_reference_radians() const { return _firing_angle_reference_radians; }

current_pu_t sfc_current_controller::get_current_reference_limited_pu() const { return _current_reference_limited_pu; }

double sfc_current_controller::get_cos_alpha_clamped() const { return _cos_alpha_clamped; }

void sfc_current_controller::recompute_internal_limits() {
    // Pre-calculate the cosine limits to avoid repeated calculations in the update loop.
    // The relationship is reversed: min angle -> max cos, max angle -> min cos.
    _cos_alpha_max = std::cos(_grid_firing_angle_min_radians.value());
    _cos_alpha_min = std::cos(_grid_firing_angle_max_radians.value());
}

}  // namespace yeet::application::sfc