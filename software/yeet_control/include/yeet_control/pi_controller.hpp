#ifndef YEET_CONTROL_PI_CONTROLLER_HPP
#define YEET_CONTROL_PI_CONTROLLER_HPP

#include <algorithm>  // For std::clamp
#include <cassert>
#include <limits>

namespace yeet::control {

/**
 * @brief Proportional-Integral (PI) controller.
 *
 * This class implements a discrete-time PI controller. It supports two methods of tuning:
 *
 * 1. Directly specifying the proportional gain (Kp) and the continuous-time integral gain (Ki).
 *    In this case, Ki (discrete) is computed as Ki_cont * Ts.
 *
 * 2. Specifying the proportional gain (Kp) and an integral time constant (Ti).
 *    The continuous-time Ki is derived as Kp/Ti.
 *
 * The controller supports output clamping (saturation) via set_output_limits().
 *
 * Common tuning notes:
 * - A lower Ti (or higher Ki) will lead to faster correction but may cause overshoot or instability.
 * - Anti-windup strategies (e.g., conditional integration, back-calculation) are recommended in applications
 *   that experience actuator saturation. They are not implemented here, but a note is provided.
 *
 * @tparam T Numeric type (should be a floating point or an arithmetic type)
 */
template <typename T, typename CoefficientT = T>
class pi_controller {
public:
    /**
     * @brief Configure the controller using Kp and continuous-time Ki.
     *
     * @param kp Proportional gain.
     * @param ki_continuous Continuous-time integral gain.
     * @param ts Sampling time (in seconds).
     *
     * The discrete-time integral gain is computed as:
     *     Ki_disc = ki_continuous * ts.
     */
    void configure(CoefficientT kp, CoefficientT ki_continuous, CoefficientT ts) {
        _kp = kp;
        _ki_cont = ki_continuous;
        _ts = ts;
        _ki_disc = ki_continuous * ts;
    }

    /**
     * @brief Configure the controller using Kp and the integral time constant.
     *
     * @param kp Proportional gain.
     * @param ti Integral time constant.
     * @param ts Sampling time (in seconds).
     *
     * Ki is computed as:
     *     Ki_continuous = kp / ti   (if ti is not zero)
     * and the discrete-time gain is:
     *     Ki_disc = Ki_continuous * ts.
     */
    void configure_with_ti(CoefficientT kp, CoefficientT ti, CoefficientT ts) {
        assert(ts > CoefficientT(0) && "Sampling time must be positive");

        _kp = kp;
        _ts = ts;
        // Protect against division by zero.
        _ki_cont = (ti != CoefficientT(0)) ? (kp / ti) : CoefficientT(0);
        _ki_disc = _ki_cont * ts;
    }

    /**
     * @brief Set output limits for saturation.
     *
     * @param min Minimum output.
     * @param max Maximum output.
     *
     * Enforcing output limits can prevent actuator saturation. Note that when saturation occurs,
     * anti-windup strategies may be necessary to prevent excessive integral buildup.
     */
    void set_output_limits(CoefficientT min, CoefficientT max) {
        if (min < max) {
            _output_min = min;
            _output_max = max;
        }
    }

    /**
     * @brief Reset the controller state by clearing the integral term.
     */
    void reset() { _integral = T{}; }

    /**
     * @brief Update the PI controller with the current error input.
     *
     * @param input The current error input.
     * @return The computed control output.
     *
     * The output is computed as:
     *     output = Kp * input + Ki_disc * (integral[previous] + current input)
     *
     * The integral term is updated in every call. If the output is limited, an anti-windup mechanism
     * (not implemented here) would normally adjust the integral term to prevent "windup."
     */
    T update(T input) {
        // Update the integral term with the current input.
        _integral += input;

        // Compute the control signal
        T output = _kp * input + _ki_disc * _integral;

        // Apply output limits (saturation)
        // A more advanced implementation could use anti-windup to adjust the _integral term.
        if constexpr (std::is_floating_point_v<T>)  // Only clamp if T is a floating point type
        {
            output = std::clamp(output, _output_min, _output_max);
        }

        return output;
    }

    // Getter methods.
    CoefficientT get_kp() const { return _kp; }
    CoefficientT get_ki_continuous() const { return _ki_cont; }
    CoefficientT get_ki_discrete() const { return _ki_disc; }
    CoefficientT get_sampling_time() const { return _ts; }
    T get_integral() const { return _integral; }

private:
    CoefficientT _kp{};       // Proportional gain.
    CoefficientT _ki_cont{};  // Continuous-time integral gain.
    CoefficientT _ts{};       // Sampling time.
    CoefficientT _ki_disc{};  // Discrete-time integral gain (computed as _ki_cont * _ts).
    T _integral{};            // Accumulated integral term.

    // Output limits for actuator saturation.
    CoefficientT _output_min{std::numeric_limits<CoefficientT>::lowest()};
    CoefficientT _output_max{std::numeric_limits<CoefficientT>::max()};
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_PI_CONTROLLER_HPP