#include "sfc/sfc_current_controller.hpp"

#include <algorithm>
#include <cmath>

#include "sfc/sfc_per_unit_system.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/limit.hpp"

namespace yeet::application::sfc {

sfc_current_controller::sfc_current_controller(duration_t sampling_time) : _pi_controller(sampling_time.value())
{
    // The PI output is a per-unit voltage. A reasonable limit for the DC voltage is ~1.5 pu.
    freeze_integrator(false);
    configure_pi_gains(0.7, duration_t(ms(100.0)));
    configure_current_limits_pu(current_pu_t(-1.5), current_pu_t(1.5));
    configure_firing_angle_limits_radians(angle_t(deg_to_rad(0.0)), angle_t(deg_to_rad(90.0)));
}

void sfc_current_controller::configure_pi_gains(real_t proportional_gain, duration_t integral_time)
{
    _pi_controller.configure_with_ti(proportional_gain, integral_time.value());
}

void sfc_current_controller::configure_current_limits_pu(current_pu_t min_current_pu, current_pu_t max_current_pu)
{
    _dc_link_current_min_pu = min_current_pu;
    _dc_link_current_max_pu = max_current_pu;
}

void sfc_current_controller::freeze_integrator(bool status) { _pi_controller.enable_integrator(!status); }

void sfc_current_controller::configure_firing_angle_limits_radians(angle_t min_angle, angle_t max_angle)
{
    _grid_firing_angle_min_radians = min_angle;
    _grid_firing_angle_max_radians = max_angle;

    // Pre-calculate the cosine limits to avoid repeated calculations in the update loop.
    // The relationship is reversed: min angle -> max cos, max angle -> min cos.
    _cos_alpha_max = std::cos(_grid_firing_angle_min_radians.value());
    _cos_alpha_min = std::cos(_grid_firing_angle_max_radians.value());
}

void sfc_current_controller::reset()
{
    _pi_controller.reset();
    _cos_alpha_clamped = 0.0;
    _cos_alpha_unclamped = 0.0;
    _firing_angle_reference_radians = angle_t{deg_to_rad(90.0)};  // Reset to 90 degrees
    _current_reference_limited_pu = current_pu_t{0.0};
    _current_error_pu = current_pu_t{0.0};
    _pi_output = real_t{0.0};
    _is_reference_current_limited = false;
    _is_current_measured_above_magnitude_limit = false;
}

void sfc_current_controller::update(current_pu_t dc_link_current_reference_pu,
    current_pu_t dc_link_current_measured,
    voltage_pu_t grid_side_voltage_magnitude,
    voltage_pu_t machine_side_voltage_magnitude)
{
    // Step 1: Saturate the incoming current reference to its absolute limits and check if limited
    const auto limit_result = yeet::control::limit<current_pu_t>::range_with_upper_limit_status(
        dc_link_current_reference_pu, _dc_link_current_min_pu, _dc_link_current_max_pu);
    _is_reference_current_limited = limit_result.was_limited;

    _current_reference_limited_pu = limit_result.value;

    // Step 2: Check if measured current exceeds the magnitude limit
    _is_current_measured_above_magnitude_limit =
        yeet::control::limit<current_pu_t>::is_above_upper_limit(dc_link_current_measured, _dc_link_current_max_pu);

    // Step 3: Calculate the error signal using the limited reference and store it
    _current_error_pu = _current_reference_limited_pu - dc_link_current_measured;

    // Step 4: Update the PI controller to get the corrective control voltage and store it
    _pi_output = _pi_controller.update(_current_error_pu.value());

    // Step 5: Add feedforward compensation (V_ctrl_total = V_ctrl_PI + V_di_est)
    const voltage_pu_t compensated_control_voltage = voltage_pu_t{_pi_output} + machine_side_voltage_magnitude;

    // Step 6: Linearize by normalizing with V_do to get cos(alpha) command
    // Avoid division by zero or small numbers for stability.
    const voltage_pu_t saturated_grid_side_voltage_magnitude = max(grid_side_voltage_magnitude, voltage_pu_t{0.2});

    _cos_alpha_unclamped = compensated_control_voltage / saturated_grid_side_voltage_magnitude;

    // Step 7: Clamp the cosine of the firing angle based on pre-calculated limits
    _cos_alpha_clamped = std::clamp(_cos_alpha_unclamped, _cos_alpha_min, _cos_alpha_max);

    // Step 8: Compute the final angle and store it
    _firing_angle_reference_radians = angle_t{std::acos(_cos_alpha_clamped)};
}

void sfc_current_controller::update(current_pu_t dc_link_current_reference_pu,
    current_pu_t dc_link_current_measured,
    voltage_pu_t grid_side_voltage_magnitude,
    voltage_pu_t machine_side_voltage_magnitude,
    const voltage_t& machine_side_base_voltage,
    const precomputed_divisor<voltage_t>& grid_side_base_voltage)
{
    // Reflect the machine-side inverter back-EMF to grid-side per-unit
    const voltage_pu_t inverter_back_emf_grid_reflected_pu = reflect_machine_to_grid_voltage_pu(
        machine_side_voltage_magnitude, machine_side_base_voltage, grid_side_base_voltage);

    // Call the main update method with the reflected voltage
    update(dc_link_current_reference_pu, dc_link_current_measured, grid_side_voltage_magnitude,
        inverter_back_emf_grid_reflected_pu);
}

angle_t sfc_current_controller::get_firing_angle_reference_radians() const { return _firing_angle_reference_radians; }

current_pu_t sfc_current_controller::get_current_reference_limited_pu() const { return _current_reference_limited_pu; }

real_t sfc_current_controller::get_cos_alpha_clamped() const { return _cos_alpha_clamped; }

current_pu_t sfc_current_controller::get_current_error_pu() const { return _current_error_pu; }

real_t sfc_current_controller::get_pi_output() const { return _pi_output; }

bool sfc_current_controller::is_saturated() const
{
    // The controller is saturated if the clamped value differs from the unclamped command
    // This provides accurate saturation detection by comparing the actual values
    constexpr real_t tolerance = 1e-10;
    return std::abs(_cos_alpha_clamped - _cos_alpha_unclamped) > tolerance;
}

bool sfc_current_controller::is_reference_current_limited() const
{
    // Return the stored boolean flag that was updated during the last update() call
    return _is_reference_current_limited;
}

bool sfc_current_controller::is_current_measured_above_magnitude_limit() const
{
    // Return the stored boolean flag that was updated during the last update() call
    return _is_current_measured_above_magnitude_limit;
}

}  // namespace yeet::application::sfc