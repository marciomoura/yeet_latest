#include "yeet_control/pi_controller.hpp"

#include <algorithm>  // For std::clamp
#include <cassert>

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/type_traits.hpp"

namespace yeet::control {

template <typename T>
void pi_controller<T>::configure(value_type kp, value_type ki_continuous)
{
    _kp = kp;
    _ki = ki_continuous;
}

template <typename T>
void pi_controller<T>::configure_with_ti(value_type kp, value_type ti)
{
    assert(ti > value_type{0} && "Integral time constant must be positive");

    _kp = kp;
    _ki = (ti >= value_type{0}) ? (kp / ti) : value_type{0};
}

template <typename T>
void pi_controller<T>::set_output_limits(value_type min, value_type max)
{
    if (min < max) {
        _output_min = min;
        _output_max = max;
        _has_output_limits = true;
    }
}

template <typename T>
void pi_controller<T>::enable_output_limits(bool enable)
{
    _has_output_limits = enable;
}

template <typename T>
void pi_controller<T>::set_antiwindup_gain(value_type kc)
{
    assert(kc >= value_type{0} && "Anti-windup gain must be non-negative");
    if (kc < value_type{0}) {
        kc = value_type{0};  // Ensure non-negative gain
        return;
    }

    _kc = kc;
}

template <typename T>
void pi_controller<T>::enable_integrator_clamping(bool enable)
{
    _integrator_clamping_enabled = enable;
}

template <typename T>
void pi_controller<T>::enable_integrator(bool enable)
{
    _integrator_enabled = enable;
}

template <typename T>
bool pi_controller<T>::is_output_saturated() const
{
    return _output_saturated;
}

template <typename T>
T pi_controller<T>::get_saturation_error() const
{
    return _saturation_error;
}

template <typename T>
void pi_controller<T>::reset(T value)
{
    _integrator.reset(value);
    _output_saturated = false;
    _saturation_error = T{};
}

template <typename T>
void pi_controller<T>::preset_for_bumpless_transfer(T desired_output, T current_error)
{
    // Calculate integrator preset: I = desired_output - Kp * current_error
    T integrator_preset = desired_output - _kp * current_error;

    // Clamp to output limits if enabled
    if (_has_output_limits) {
        integrator_preset = clamp_coordinate_output(integrator_preset);
    }

    // Reset integrator with calculated preset value
    _integrator.reset(integrator_preset);
    _output_saturated = false;
    _saturation_error = T{};
}

template <typename T>
T pi_controller<T>::update(T input)
{
    // Compute the proportional term
    T const proportional_term = _kp * input;

    // Compute the total output (before saturation)
    T unsaturated_output = proportional_term + _integrator.get_output();

    // Apply output limits and anti-windup if enabled
    T saturated_output = unsaturated_output;
    bool is_saturated = false;

    if (_has_output_limits) {
        saturated_output = clamp_coordinate_output(unsaturated_output);
        is_saturated = !are_equal(saturated_output, unsaturated_output);
    }

    // Store saturation state for diagnostics
    _output_saturated = is_saturated;
    _saturation_error = saturated_output - unsaturated_output;

    // Update integral term only if integrator is enabled
    if (!_integrator_enabled) {
        return saturated_output;  // Short-circuit if integrator is disabled
    }

    // Conditional integration (clamping) anti-windup
    if (_integrator_clamping_enabled && _output_saturated) {
        // Integrator is frozen when output is saturated
        return saturated_output;
    }

    // Integrator update with back-calculation anti-windup logic
    // If _kc = 0, no anti-windup is applied
    // If saturation occurs, the anti-windup logic reduces the integral term
    // If saturation does not occur, the integral term is only affected by the integral term
    T const integral_term = (_ki * input + _kc * _saturation_error);
    _integrator.update(integral_term);

    return saturated_output;  // Short-circuit if integrator is disabled
}

template <typename T>
T pi_controller<T>::clamp_coordinate_output(const T& output) const
{
    if constexpr (is_coordinate_frame_v<T>) {
        T result = output;
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = std::clamp(result[i], _output_min, _output_max);
        }
        return result;
    }
    else {
        return std::clamp(output, _output_min, _output_max);
    }
}

template <typename T>
bool pi_controller<T>::are_equal(const T& a, const T& b) const
{
    constexpr value_type tolerance = value_type(1e-9);

    if constexpr (is_coordinate_frame_v<T>) {
        for (size_t i = 0; i < a.size(); ++i) {
            if (std::abs(a[i] - b[i]) > tolerance) {
                return false;
            }
        }
        return true;
    }
    else {
        return std::abs(a - b) <= tolerance;
    }
}

// Explicit instantiations
template class pi_controller<float>;
template class pi_controller<abc<float>>;
template class pi_controller<dq<float>>;
template class pi_controller<alphabeta<float>>;

template class pi_controller<double>;
template class pi_controller<abc<double>>;
template class pi_controller<dq<double>>;
template class pi_controller<alphabeta<double>>;

}  // namespace yeet::control