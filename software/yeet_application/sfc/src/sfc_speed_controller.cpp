#include "sfc/sfc_speed_controller.hpp"

#include <algorithm>

namespace yeet::application::sfc {

sfc_speed_controller::sfc_speed_controller(duration_t sampling_time)
    : _sampling_time(sampling_time),
      _pi_controller(sampling_time.value()),
      _speed_ref_derivative(sampling_time.value(), 0.0f)
{
    // Enable internal anti-windup: stops integration when PI output hits torque limits
    _pi_controller.enable_integrator_clamping(true);

    // Optional: Enable back-calculation anti-windup for faster recovery from saturation
    // Typical value: kc = 1/Ti, but this is set automatically when using configure_with_ti
    // _pi_controller.set_antiwindup_gain(1.0);

    _pi_controller.configure_with_ti(3.0, 1.0);
}

void sfc_speed_controller::configure_pi_gains(real_t kp, duration_t ti)
{
    assert(kp > 0.0 && "Proportional gain must be positive.");
    assert(ti.value() > 0.0 && "Integral time must be positive.");
    if (kp <= 0.0 || ti.value() <= 0.0) {
        return;
    }

    _pi_controller.configure_with_ti(kp, ti.value());
}

void sfc_speed_controller::configure_torque_limits_pu(torque_pu_t min_pu, torque_pu_t max_pu)
{
    assert(min_pu.value() < max_pu.value() && "Minimum torque limit must be less than maximum.");
    if (max_pu.value() <= min_pu.value()) {
        return;
    }

    // Store the limits for later use with dynamic braking limit
    _torque_min_limit = min_pu.value();
    _torque_max_limit = max_pu.value();

    _pi_controller.set_output_limits(min_pu.value(), max_pu.value());
    _pi_controller.enable_output_limits(true);
}

void sfc_speed_controller::configure_feedforward_pu(inertia_constant_pu_t inertia_pu)
{
    assert(inertia_pu >= 0.0 && "Inertia must be non-negative.");
    if (_inertia_pu > 0.0) {
        _inertia_pu = inertia_pu;
    }
}

void sfc_speed_controller::update(speed_pu_t speed_ref_pu, speed_pu_t speed_measured_pu)
{
    // Step 1: Store the speed reference directly (no rate limiting - upstream generator handles this)
    _processed_speed_ref_pu = speed_ref_pu;

    // Step 2: Update derivative calculator with the speed reference
    _speed_ref_derivative.update(_processed_speed_ref_pu.value());

    // Step 3: Calculate speed error
    const speed_pu_t speed_error_pu = _processed_speed_ref_pu - speed_measured_pu;

    // Step 4: Update PI controller and store the output
    _pi_output = _pi_controller.update(speed_error_pu.value());

    // Step 5: Calculate optional feed-forward term using the derivative class
    const real_t speed_derivative = _speed_ref_derivative.get_output();
    const real_t speed_derivative_speed_feedforward = _inertia_pu * speed_derivative;

    // Step 6: Sum PI and feed-forward outputs to get unclamped torque reference
    const real_t total_torque_ref = _pi_output + speed_derivative_speed_feedforward;
    _torque_ref_unclamped = torque_pu_t{total_torque_ref};

    // Step 7: Apply final clamping with configured limits
    _torque_ref_pu = torque_pu_t{std::clamp(total_torque_ref, _torque_min_limit, _torque_max_limit)};
}

torque_pu_t sfc_speed_controller::get_torque_ref_pu() const { return _torque_ref_pu; }

speed_pu_t sfc_speed_controller::get_processed_speed_ref_pu() const { return _processed_speed_ref_pu; }

real_t sfc_speed_controller::get_pi_output() const { return _pi_output; }

bool sfc_speed_controller::is_saturated() const
{
    // Check if the PI controller is saturated by using the built-in saturation detection
    return _pi_controller.is_output_saturated();
}

void sfc_speed_controller::reset(speed_pu_t initial_speed_pu)
{
    _pi_controller.reset();
    _speed_ref_derivative.reset(initial_speed_pu.value());
    _processed_speed_ref_pu = initial_speed_pu;
    _torque_ref_pu = torque_pu_t{0.0};
    _pi_output = real_t{0.0};
    _torque_ref_unclamped = torque_pu_t{0.0};
}

void sfc_speed_controller::freeze_integrator(bool status) { _pi_controller.enable_integrator(!status); }

}  // namespace yeet::application::sfc