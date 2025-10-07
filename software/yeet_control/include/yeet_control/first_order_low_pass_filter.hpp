#ifndef YEET_CONTROL_FIRST_ORDER_LOW_PASS_FILTER_HPP
#define YEET_CONTROL_FIRST_ORDER_LOW_PASS_FILTER_HPP

#include <cassert>
#include <cmath>
#include <type_traits>

#include "constants.hpp"
#include "type_traits.hpp"

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
 *
 * Implementation Note:
 * All internal arithmetic is performed in double precision to maintain numerical
 * stability, especially for small coefficient values. The interface accepts and
 * returns the template type T, with automatic conversions handled internally.
 */
template <typename T>
class first_order_low_pass_filter {
public:
    using type = fundamental_type_t<T>;
    static_assert(std::is_floating_point_v<type>, "Fundamental type must be floating point");

    explicit first_order_low_pass_filter(double sampling_period)
        : _prev_input_internal(0.0), _output_internal(0.0), _ts(sampling_period)
    {
    }

    void configure(type cutoff_freq)
    {
        const double cutoff_freq_double = static_cast<double>(cutoff_freq);

        // Check for negative frequency
        if (cutoff_freq_double <= 0.0) {
            assert(false && "Cutoff frequency must be positive");
            return;
        }

        // Check Nyquist criterion
        const double nyquist_freq = 0.5 / _ts;
        if (cutoff_freq_double >= nyquist_freq) {
            assert(false && "Cutoff frequency must be less than Nyquist frequency (fs/2)");
            return;
        }

        // Convert Hz to radians/second and pre-warp (all in double precision)
        const double omega_c = 2.0 * pi * cutoff_freq_double;
        const double omega_w = 2.0 / _ts * std::tan(omega_c * _ts * 0.5);

        // Compute filter coefficients (stored as double)
        const double denom = omega_w * _ts + 2.0;
        _b0 = omega_w * _ts / denom;
        _b1 = _b0;  // Symmetric for first-order lowpass
        _a1 = (omega_w * _ts - 2.0) / denom;
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
    static constexpr T calculate_cutoff_frequency(T time_constant)
    {
        const double tc_double = static_cast<double>(static_cast<type>(time_constant));
        if (tc_double <= 0.0) {
            return T(type(0));
        }
        const double result = 1.0 / (2.0 * pi * tc_double);
        return T(static_cast<type>(result));
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
    static constexpr T calculate_settling_time(T cutoff_freq, T percent_of_final_value)
    {
        const double fc_double = static_cast<double>(static_cast<type>(cutoff_freq));
        const double percent_double = static_cast<double>(static_cast<type>(percent_of_final_value));

        if (fc_double <= 0.0 || percent_double <= 0.0 || percent_double >= 100.0) {
            return T(type(0));
        }

        // Calculate time constant (τ = 1/(2πfc))
        const double time_constant = 1.0 / (2.0 * pi * fc_double);

        // Calculate settling time: t = -τ * ln(1 - p/100)
        const double result = -time_constant * std::log(1.0 - percent_double / 100.0);
        return T(static_cast<type>(result));
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
     * All arithmetic is performed in double precision internally for numerical stability.
     *
     * @param input Current input sample x[n]
     * @return Filtered output sample y[n]
     */
    T update(T input)
    {
        // Convert input to double for internal calculations
        const double input_double = static_cast<double>(static_cast<type>(input));

        // Perform all arithmetic in double precision
        const double output_double = -_a1 * _output_internal + _b0 * input_double + _b1 * _prev_input_internal;

        // Update internal state (double precision)
        _prev_input_internal = input_double;
        _output_internal = output_double;

        // Convert back to output type
        return T(static_cast<type>(output_double));
    }

    T get_output() const { return T(static_cast<type>(_output_internal)); }

    /**
     * Reset filter state variables to specified value
     */
    void reset(type value = type(0))
    {
        const double value_double = static_cast<double>(value);
        _prev_input_internal = value_double;
        _output_internal = value_double;
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
    T get_magnitude_response(T freq) const
    {
        const double freq_double = static_cast<double>(static_cast<type>(freq));
        const double omega = compute_omega(freq_double);
        const double omega_t = omega * _ts;
        const double cos_omega_t = std::cos(omega_t);

        // Compute numerator and denominator (all in double precision)
        const double num = _b0 * _b0 + _b1 * _b1 + 2.0 * _b0 * _b1 * cos_omega_t;
        const double den = 1.0 + _a1 * _a1 + 2.0 * _a1 * cos_omega_t;

        const double result = std::sqrt(num / den);
        return T(static_cast<type>(result));
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
    T get_magnitude_response_db(T freq) const
    {
        const double mag = static_cast<double>(static_cast<type>(get_magnitude_response(freq)));
        const double result = 20.0 * std::log10(mag);
        return T(static_cast<type>(result));
    }

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
    T get_phase_response(T freq) const
    {
        const double freq_double = static_cast<double>(static_cast<type>(freq));
        const double omega = compute_omega(freq_double);
        const double omega_t = omega * _ts;
        const double sin_omega_t = std::sin(omega_t);
        const double cos_omega_t = std::cos(omega_t);

        // Compute numerator phase
        const double num_imag = -(_b0 - _b1) * sin_omega_t;
        const double num_real = (_b0 + _b1) + (_b0 - _b1) * cos_omega_t;
        const double num_phase = std::atan2(num_imag, num_real);

        // Compute denominator phase
        const double den_imag = -_a1 * sin_omega_t;
        const double den_real = 1.0 + _a1 * cos_omega_t;
        const double den_phase = std::atan2(den_imag, den_real);

        const double result = num_phase - den_phase;
        return T(static_cast<type>(result));
    }

    /**
     * Convert phase response from radians to degrees
     * @param freq Frequency in Hz
     * @return Phase response in degrees
     */
    T get_phase_response_degrees(T freq) const
    {
        const double phase_rad = static_cast<double>(static_cast<type>(get_phase_response(freq)));
        const double result = phase_rad * 180.0 / pi;
        return T(static_cast<type>(result));
    }

    // Getter methods (return external type, coefficients are double internally)
    type get_sampling_time() const { return static_cast<type>(_ts); }
    type get_feedback_coeff_a1() const { return static_cast<type>(_a1); }
    type get_feedforward_coeff_b0() const { return static_cast<type>(_b0); }
    type get_feedforward_coeff_b1() const { return static_cast<type>(_b1); }

private:
    /**
     * Convert frequency from Hz to radians/second
     * ω = 2π * f
     */
    static constexpr double compute_omega(double freq) { return 2.0 * pi * freq; }

    // All coefficients and internal state in double precision
    double _a1{0.0};                   // Feedback coefficient (denominator)
    double _b0{0.0};                   // Current input coefficient (numerator)
    double _b1{0.0};                   // Previous input coefficient (numerator)
    double _prev_input_internal{0.0};  // Previous input sample x[n-1]
    double _output_internal{0.0};      // Previous output sample y[n-1]
    double _ts{0.0};                   // Sampling period in seconds
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_FIRST_ORDER_LOW_PASS_FILTER_HPP