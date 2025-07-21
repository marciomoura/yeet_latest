#ifndef YEET_CONTROL_PI_CONTROLLER_HPP
#define YEET_CONTROL_PI_CONTROLLER_HPP

#include <cassert>
#include <limits>

#include "integrator.hpp"
#include "type_traits.hpp"

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
template <typename T>
class pi_controller {
public:
    using value_type = fundamental_type_t<T>;
    static_assert(std::is_floating_point_v<value_type>, "Value type must be a floating point type");

    pi_controller() = default;
    pi_controller(value_type ts) : _integrator(ts) {}

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
    void configure(value_type kp, value_type ki_continuous);

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
    void configure_with_ti(value_type kp, value_type ti);

    /**
     * @brief Set output limits for saturation.
     *
     * @param min Minimum output.
     * @param max Maximum output.
     *
     * Enforcing output limits can prevent actuator saturation. Anti-windup logic
     * is automatically applied when saturation occurs.
     */
    void set_output_limits(value_type min, value_type max);

    /**
     * @brief Enable or disable output limits.
     *
     * @param enable True to enable output limits, false to disable.
     */
    void enable_output_limits(bool enable);

    /**
     * @brief Set the anti-windup gain.
     *
     * @param kc Anti-windup gain (typically 1/Ti or similar).
     *
     * The anti-windup mechanism uses back-calculation to prevent integral windup
     * when the output saturates. A typical value is kc = 1/Ti.
     */
    void set_antiwindup_gain(value_type kc);

    /**
     * @brief Enable or disable integrator clamping as an anti-windup method.
     *
     * @param enable True to enable, false to disable.
     */
    void enable_integrator_clamping(bool enable);

    /**
     * @brief Enable or disable the integrator.
     *
     * @param enable True to enable the integrator, false to disable.
     *
     * When disabled, the integral term is not updated and the controller
     * operates as a pure proportional controller.
     */
    void enable_integrator(bool enable);

    /**
     * @brief Check if the output was saturated in the last update.
     *
     * @return True if the output was saturated, false otherwise.
     */
    bool is_output_saturated() const;

    /**
     * @brief Get the saturation error from the last update.
     *
     * @return The difference between saturated and unsaturated output.
     */
    T get_saturation_error() const;

    /**
     * @brief Reset the controller state by clearing the integral term.
     * @param value Initial value to reset the integral term to (default is 0).
     */
    void reset(T value = T{});

    /**
     * @brief Update the PI controller with the current error input.
     *
     * @param input The current error input.
     * @return The computed control output.
     *
     * The output is computed as:
     *     output = Kp * input + Ki_disc * integral
     *
     * The integral term is updated only if the integrator is enabled.
     * Anti-windup logic is applied when output saturation occurs.
     */
    T update(T input);

    // Getter methods.
    value_type get_kp() const { return _kp; }
    value_type get_ki() const { return _ki; }
    value_type get_sampling_time() const { return _integrator.get_sampling_time(); }
    T get_integral() const { return _integrator.get_output(); }
    bool is_integrator_enabled() const { return _integrator_enabled; }
    bool has_output_limits() const { return _has_output_limits; }
    value_type get_antiwindup_gain() const { return _kc; }

private:
    /**
     * @brief Clamp coordinate frame output to limits.
     */
    T clamp_coordinate_output(const T& output) const;

    /**
     * @brief Check if two coordinate frame values are equal (within tolerance).
     */
    bool are_equal(const T& a, const T& b) const;

    value_type _kp{};  // Proportional gain.
    value_type _ki{};  // Continuous-time integral gain.

    integrator<T> _integrator;  // Integrator for the integral term.

    // Output limits for actuator saturation.
    value_type _output_min{std::numeric_limits<value_type>::lowest()};
    value_type _output_max{std::numeric_limits<value_type>::max()};
    bool _has_output_limits{false};  // Flag to enable/disable output limits.

    // Anti-windup parameters.
    value_type _kc{value_type{1}};  // Anti-windup gain.
    bool _integrator_clamping_enabled{false};

    // Integrator control.
    bool _integrator_enabled{true};  // Flag to enable/disable integrator.

    // Saturation diagnostics.
    bool _output_saturated{false};  // Flag indicating if output was saturated in last update.
    T _saturation_error{};          // Saturation error from last update.
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_PI_CONTROLLER_HPP