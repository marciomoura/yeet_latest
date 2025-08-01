#include "sfc/sfc_speed_controller.hpp"

#include <algorithm>

namespace yeet::application::sfc {

sfc_speed_controller::sfc_speed_controller(duration_t sampling_time)
    : _sampling_time(sampling_time),
      _pi_controller(sampling_time.value()),
      _speed_ref_rate_limiter(1.0, sampling_time.value()),
      _speed_ref_derivative(sampling_time.value(), 0.0F) {
    _pi_controller.enable_integrator_clamping(true);
}

void sfc_speed_controller::configure_pi_gains(real_t kp, duration_t ti) {
    assert(kp > 0.0 && "Proportional gain must be positive.");
    assert(ti.value() > 0.0 && "Integral time must be positive.");
    if (kp <= 0.0 || ti.value() <= 0.0) {
        return;
    }

    _pi_controller.configure_with_ti(kp, ti.value());
}

void sfc_speed_controller::configure_torque_limits_pu(torque_pu_t min_pu, torque_pu_t max_pu) {
    // assert(min_pu.value() < max_pu.value() && "Minimum torque limit must be less than maximum.");
    // assert(min_pu.value() >= 0.0 && "Minimum torque limit must be non-negative.");
    // if (min_pu.value() < 0.0 || max_pu.value() <= min_pu.value()) {
    //     return;
    // }

    _pi_controller.set_output_limits(min_pu.value(), max_pu.value());
    _pi_controller.enable_output_limits(true);
}

void sfc_speed_controller::configure_speed_reference_limits_pu(speed_pu_t min_pu, speed_pu_t max_pu) {
    assert(min_pu.value() < max_pu.value() && "Minimum speed reference must be less than maximum.");
    assert(min_pu.value() >= 0.0 && "Minimum speed reference must be non-negative.");

    if (min_pu.value() < 0.0 || max_pu.value() <= min_pu.value()) {
        return;
    }

    _min_speed_ref_pu = min_pu;
    _max_speed_ref_pu = max_pu;
}

void sfc_speed_controller::configure_speed_reference_rate_limit(real_t rate_limit_pu_per_second) {
    assert(rate_limit_pu_per_second > 0.0 && "Rate limit must be positive.");
    if (rate_limit_pu_per_second <= 0.0) {
        return;
    }

    _speed_ref_rate_limiter.configure(rate_limit_pu_per_second);
}

void sfc_speed_controller::configure_feedforward_pu(real_t inertia_pu) {
    assert(inertia_pu >= 0.0 && "Inertia must be non-negative.");
    if (_inertia_pu > 0.0) {
        _inertia_pu = inertia_pu;
    }
}

void sfc_speed_controller::update(speed_pu_t speed_ref_pu, speed_pu_t speed_measured_pu) {
    // Step 1: Clamp the raw speed reference
    const real_t clamped_speed_ref =
        std::clamp(speed_ref_pu.value(), _min_speed_ref_pu.value(), _max_speed_ref_pu.value());

    // Step 2: Apply rate limiting to the clamped reference
    _speed_ref_rate_limiter.update(clamped_speed_ref);
    _processed_speed_ref_pu = speed_pu_t{_speed_ref_rate_limiter.get_output()};

    // Step 2.5: Update derivative calculator with the processed speed reference
    _speed_ref_derivative.update(_processed_speed_ref_pu.value());

    // Step 3: Calculate speed error
    const real_t speed_error_pu = _processed_speed_ref_pu.value() - speed_measured_pu.value();

    // Step 4: Update PI controller
    const real_t pi_output = _pi_controller.update(speed_error_pu);

    // Step 5: Calculate optional feed-forward term using the derivative class
    const real_t speed_derivative = _speed_ref_derivative.get_output();
    const real_t speed_derivative_speed_feedforward = _inertia_pu * speed_derivative;

    // Step 6: Sum PI and feed-forward outputs.
    const real_t total_torque_ref = pi_output + speed_derivative_speed_feedforward;
    _torque_ref_pu = torque_pu_t{total_torque_ref};
}

torque_pu_t sfc_speed_controller::get_torque_ref_pu() const { return _torque_ref_pu; }

speed_pu_t sfc_speed_controller::get_processed_speed_ref_pu() const { return _processed_speed_ref_pu; }

void sfc_speed_controller::reset(speed_pu_t initial_speed_pu) {
    _pi_controller.reset();
    _speed_ref_rate_limiter.reset(initial_speed_pu.value());
    _speed_ref_derivative.reset(initial_speed_pu.value());
    _processed_speed_ref_pu = initial_speed_pu;
    _torque_ref_pu = torque_pu_t{0.0};
}

}  // namespace yeet::application::sfc