#ifndef YEET_CONTROL_FIRST_ORDER_LOW_PASS_FILTER_HPP
#define YEET_CONTROL_FIRST_ORDER_LOW_PASS_FILTER_HPP

#include <cassert>
#include <cmath>
#include <type_traits>

#include "constants.hpp"

namespace yeet::control {

/**
 * First-order low-pass filter implementation.
 *
 * Mathematical Background:
 * ----------------------
 * The continuous-time transfer function of a first-order low-pass filter is:
 *     H(s) = ωc / (s + ωc)
 * where ωc is the cutoff frequency in radians/second.
 *
 * For digital implementation, we use the bilinear transform (Tustin's method):
 *     s = (2/Ts) * (z - 1)/(z + 1)
 *
 * This gives us the discrete-time transfer function:
 *     H(z) = (b0 * z + b1) / (z + a1)
 *
 * The difference equation implemented is:
 *     y[n] = -a1 * y[n-1] + b0 * x[n] + b1 * x[n-1]
 *
 * Frequency Warping:
 * To compensate for frequency warping in the bilinear transform, we pre-warp
 * the cutoff frequency using:
 *     ωw = (2/Ts) * tan(ωc * Ts/2)
 */
template <typename T>
class first_order_low_pass_filter {
    static_assert(std::is_floating_point_v<T>, "T must be a floating point type");

public:
    /**
     * Initialize filter with sampling period.
     * @param sampling_period Time between samples in seconds
     */
    explicit first_order_low_pass_filter(T sampling_period) : _prev_input(T(0)), _output(T(0)), _ts(sampling_period) {}

    /**
     * Configure filter with cutoff frequency.
     *
     * Mathematical Process:
     * 1. Pre-warp the cutoff frequency: ωw = (2/Ts) * tan(ωc * Ts/2)
     * 2. Apply bilinear transform and solve for coefficients:
     *    b0 = b1 = ωw * Ts / (ωw * Ts + 2)
     *    a1 = (ωw * Ts - 2) / (ωw * Ts + 2)
     *
     * @param cutoff_freq Cutoff frequency in Hz (-3dB point)
     */
    void configure(T cutoff_freq) {
        // Check for negative frequency
        if (cutoff_freq <= T(0)) {
            assert(false && "Cutoff frequency must be positive");
            return;
        }

        // Check Nyquist criterion
        const T nyquist_freq = T(0.5) / _ts;
        if (cutoff_freq >= nyquist_freq) {
            assert(false && "Cutoff frequency must be less than Nyquist frequency (fs/2)");
            return;
        }

        // Convert Hz to radians/second and pre-warp
        const T omega_c = T(2.0) * pi<T> * cutoff_freq;
        const T omega_w = T(2.0) / _ts * std::tan(omega_c * _ts * T(0.5));

        // Compute filter coefficients
        const T denom = omega_w * _ts + T(2.0);
        _b0 = omega_w * _ts / denom;
        _b1 = _b0;  // Symmetric for first-order lowpass
        _a1 = (omega_w * _ts - T(2.0)) / denom;
    }

    /**
     * Calculate cutoff frequency from time constant.
     *
     * For a first-order system:
     * ωc = 1/τ
     * fc = 1/(2πτ)
     *
     * where:
     * ωc = cutoff frequency in radians/second
     * fc = cutoff frequency in Hz
     * τ = time constant in seconds
     *
     * @param time_constant Time constant in seconds
     * @return Cutoff frequency in Hz
     */
    static constexpr T calculate_cutoff_frequency(T time_constant) {
        if (time_constant <= T(0)) {
            return T(0);
        }
        return T(1.0) / (T(2.0) * pi<T> * time_constant);
    }

    /**
     * Calculate time to reach steady state within a specified percentage.
     *
     * For a first-order system, the response is:
     * y(t) = A(1 - e^(-t/τ))
     *
     * To reach p% of final value:
     * t = -τ * ln(1 - p/100)
     *
     * Common values:
     * - 63.2% : t = τ
     * - 86.5% : t = 2τ
     * - 95.0% : t = 3τ
     * - 98.2% : t = 4τ
     * - 99.3% : t = 5τ
     *
     * @param cutoff_freq Cutoff frequency in Hz
     * @param percent_of_final_value Percentage of final value (0-100)
     * @return Time in seconds to reach specified percentage
     */
    static constexpr T calculate_settling_time(T cutoff_freq, T percent_of_final_value) {
        if (cutoff_freq <= T(0) || percent_of_final_value <= T(0) || percent_of_final_value >= T(100)) {
            return T(0);
        }

        // Calculate time constant (τ = 1/(2πfc))
        const T time_constant = T(1.0) / (T(2.0) * pi<T> * cutoff_freq);

        // Calculate settling time: t = -τ * ln(1 - p/100)
        return -time_constant * std::log(T(1.0) - percent_of_final_value / T(100.0));
    }

    /**
     * Process input sample through filter.
     *
     * Implements the difference equation:
     *     y[n] = -a1 * y[n-1] + b0 * x[n] + b1 * x[n-1]
     *
     * This equation represents a recursive filter where:
     * - y[n] is the current output
     * - y[n-1] is the previous output
     * - x[n] is the current input
     * - x[n-1] is the previous input
     *
     * @param input Current input sample x[n]
     * @return Filtered output sample y[n]
     */
    T update(T input) {
        const T output = -_a1 * _output + _b0 * input + _b1 * _prev_input;

        // Update state variables
        _prev_input = input;
        _output = output;

        return output;
    }

    T get_output() const { return _output; }

    /**
     * Reset filter state variables to zero
     */
    void reset() {
        _prev_input = T(0);
        _output = T(0);
    }

    /**
     * Calculate magnitude response at given frequency.
     *
     * For a digital filter H(z) = (b0*z + b1)/(z + a1),
     * the magnitude response is calculated by evaluating |H(e^(jωT))|:
     *
     * |H(ω)| = sqrt[(b0²+b1²+2*b0*b1*cos(ωT)) / (1+a1²+2*a1*cos(ωT))]
     *
     * @param freq Frequency in Hz
     * @return Magnitude response (gain factor, not in dB)
     */
    T get_magnitude_response(T freq) const {
        const T omega = compute_omega(freq);
        const T omega_t = omega * _ts;
        const T cos_omega_t = std::cos(omega_t);

        // Compute numerator and denominator
        const T num = _b0 * _b0 + _b1 * _b1 + T(2.0) * _b0 * _b1 * cos_omega_t;
        const T den = T(1.0) + _a1 * _a1 + T(2.0) * _a1 * cos_omega_t;

        return std::sqrt(num / den);
    }

    /**
     * Calculate magnitude response in decibels.
     *
     * Converts magnitude response to decibels using:
     * H_dB(ω) = 20 * log10(|H(ω)|)
     *
     * @param freq Frequency in Hz
     * @return Magnitude response in dB
     */
    T get_magnitude_response_db(T freq) const { return T(20.0) * std::log10(get_magnitude_response(freq)); }

    /**
     * Calculate phase response at given frequency.
     *
     * For H(z) = (b0*z + b1)/(z + a1), the phase response is:
     * ∠H(ω) = atan2(Im{num}, Re{num}) - atan2(Im{den}, Re{den})
     * where:
     * num = (b0+b1) + (b0-b1)*cos(ωT) - j(b0-b1)*sin(ωT)
     * den = 1 + a1*cos(ωT) - j*a1*sin(ωT)
     *
     * @param freq Frequency in Hz
     * @return Phase response in radians
     */
    T get_phase_response(T freq) const {
        const T omega = compute_omega(freq);
        const T omega_t = omega * _ts;
        const T sin_omega_t = std::sin(omega_t);
        const T cos_omega_t = std::cos(omega_t);

        // Compute numerator phase
        const T num_imag = -(_b0 - _b1) * sin_omega_t;
        const T num_real = (_b0 + _b1) + (_b0 - _b1) * cos_omega_t;
        const T num_phase = std::atan2(num_imag, num_real);

        // Compute denominator phase
        const T den_imag = -_a1 * sin_omega_t;
        const T den_real = T(1.0) + _a1 * cos_omega_t;
        const T den_phase = std::atan2(den_imag, den_real);

        return num_phase - den_phase;
    }

    /**
     * Convert phase response from radians to degrees
     * @param freq Frequency in Hz
     * @return Phase response in degrees
     */
    T get_phase_response_degrees(T freq) const { return get_phase_response(freq) * T(180.0) / pi<T>; }

    // Getter methods
    T get_sampling_time() const { return _ts; }
    T get_feedback_coeff_a1() const { return _a1; }
    T get_feedforward_coeff_b0() const { return _b0; }
    T get_feedforward_coeff_b1() const { return _b1; }

private:
    /**
     * Convert frequency from Hz to radians/second
     * ω = 2π * f
     */
    static constexpr T compute_omega(T freq) { return T(2.0) * pi<T> * freq; }

    T _a1{};        // Feedback coefficient (denominator)
    T _b0{};        // Current input coefficient (numerator)
    T _b1{};        // Previous input coefficient (numerator)
    T _prev_input;  // Previous input sample x[n-1]
    T _output;      // Previous output sample y[n-1]
    T _ts;          // Sampling period in seconds
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_FIRST_ORDER_LOW_PASS_FILTER_HPP