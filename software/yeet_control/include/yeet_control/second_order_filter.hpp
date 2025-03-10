#ifndef YEET_CONTROL_SECOND_ORDER_FILTER_HPP
#define YEET_CONTROL_SECOND_ORDER_FILTER_HPP

#include <cmath>
#include <type_traits>

#include "constants.hpp"
#include "type_traits.hpp"

namespace yeet::control {

/**
 * Base class for second-order IIR (Infinite Impulse Response) digital filters.
 *
 * Mathematical Background:
 * ----------------------
 * The general form of a second-order IIR filter in Z-domain is:
 *     H(z) = (b0 + b1*z^(-1) + b2*z^(-2)) / (1 + a1*z^(-1) + a2*z^(-2))
 *
 * This translates to the difference equation:
 *     y[n] = -a1*y[n-1] - a2*y[n-2] + b0*x[n] + b1*x[n-1] + b2*x[n-2]
 *
 * where:
 * - x[n] is the current input
 * - y[n] is the current output
 * - a1, a2 are feedback coefficients
 * - b0, b1, b2 are feedforward coefficients
 */
template <typename T>
class second_order_filter {
public:
    using type = fundamental_type_t<T>;

    static_assert(std::is_floating_point_v<type>, "Fundamental type must be floating point");

protected:
    /**
     * Protected constructor for derived classes
     * @param sampling_period Sampling period in seconds
     */
    second_order_filter(type sampling_period) : _ts(sampling_period) {}

    // Compute angular frequency in radians/second from Hz
    static constexpr type compute_omega(type freq) { return type(2.0) * pi<type> * freq; }

    // Get pre-warped frequency using bilinear transform
    type compute_warped_omega(type freq) const {
        const type omega = compute_omega(freq);
        return type(2.0) / _ts * std::tan(omega * _ts * type(0.5));
    }

public:
    /**
     * Process input sample through filter
     * @param input Current input sample x[n]
     * @return Filtered output sample y[n]
     */
    T update(T input) {
        // Implement difference equation using coefficient type conversion
        const T output = -_a1 * _y1 - _a2 * _y2 + _b0 * input + _b1 * _x1 + _b2 * _x2;

        // Update state variables
        _x2 = _x1;
        _x1 = input;
        _y2 = _y1;
        _y1 = output;

        return output;
    }

    /**
     * Get magnitude response at given frequency
     * @param freq Frequency in Hz
     * @return Magnitude response (gain factor)
     */
    type get_magnitude_response(type freq) const {
        const type omega = compute_omega(freq);
        const type omega_t = omega * _ts;
        const type cos_omega_t = std::cos(omega_t);
        const type cos_2omega_t = std::cos(type(2.0) * omega_t);

        // Compute numerator and denominator for H(e^(jωT))
        const type num = _b0 * _b0 + _b1 * _b1 + _b2 * _b2 + type(2.0) * (_b0 * _b1 + _b1 * _b2) * cos_omega_t +
                         type(2.0) * _b0 * _b2 * cos_2omega_t;

        const type den = type(1.0) + _a1 * _a1 + _a2 * _a2 + type(2.0) * (_a1 + _a1 * _a2) * cos_omega_t +
                         type(2.0) * _a2 * cos_2omega_t;

        return std::sqrt(num / den);
    }

    /**
     * Get magnitude response in decibels
     * @param freq Frequency in Hz
     * @return Magnitude response in dB
     */
    T get_magnitude_response_db(T freq) const { return T(20.0) * std::log10(get_magnitude_response(freq)); }

    /**
     * Get phase response at given frequency
     * @param freq Frequency in Hz
     * @return Phase shift in radians
     */
    type get_phase_response(type freq) const {
        const type omega = compute_omega(freq);
        const type omega_t = omega * _ts;
        const type sin_omega_t = std::sin(omega_t);
        const type sin_2omega_t = std::sin(type(2.0) * omega_t);
        const type cos_omega_t = std::cos(omega_t);
        const type cos_2omega_t = std::cos(type(2.0) * omega_t);

        // Compute numerator and denominator phases
        const type num_real = _b0 + _b1 * cos_omega_t + _b2 * cos_2omega_t;
        const type num_imag = -(_b1 * sin_omega_t + _b2 * sin_2omega_t);
        const type num_phase = std::atan2(num_imag, num_real);

        const type den_real = type(1.0) + _a1 * cos_omega_t + _a2 * cos_2omega_t;
        const type den_imag = -(_a1 * sin_omega_t + _a2 * sin_2omega_t);
        const type den_phase = std::atan2(den_imag, den_real);

        return num_phase - den_phase;
    }

    /**
     * Get phase response in degrees
     * @param freq Frequency in Hz
     * @return Phase shift in degrees
     */
    type get_phase_response_degrees(type freq) const { return get_phase_response(freq) * type(180.0) / pi<type>; }

    /**
     * Reset filter state variables
     */
    void reset() { _x1 = _x2 = _y1 = _y2 = T(0); }

    // Getter methods
    type get_sampling_time() const { return _ts; }
    type get_feedback_coeff_1() const { return _a1; }
    type get_feedback_coeff_2() const { return _a2; }
    type get_feedforward_coeff_0() const { return _b0; }
    type get_feedforward_coeff_1() const { return _b1; }
    type get_feedforward_coeff_2() const { return _b2; }

protected:
    // Filter coefficients using type
    type _a1{};  // Feedback coefficient 1
    type _a2{};  // Feedback coefficient 2
    type _b0{};  // Feedforward coefficient 0
    type _b1{};  // Feedforward coefficient 1
    type _b2{};  // Feedforward coefficient 2

    // Sampling period
    type _ts;

    // State variables using type
    T _x1{};  // Previous input x[n-1]
    T _x2{};  // Two samples ago input x[n-2]
    T _y1{};  // Previous output y[n-1]
    T _y2{};  // Two samples ago output y[n-2]
};

/**
 * Second-order low-pass filter implementation.
 *
 * Mathematical Background:
 * ----------------------
 * The continuous-time transfer function is:
 *     H(s) = ωc² / (s² + 2ζωc*s + ωc²)
 *
 * where:
 * - ωc is the cutoff frequency in radians/second
 * - ζ is the damping ratio
 */
template <typename T>
class second_order_low_pass_filter : public second_order_filter<T> {
public:
    using type = fundamental_type_t<T>;

    second_order_low_pass_filter(type sampling_period) : second_order_filter<T>(sampling_period) {}

    /**
     * Configure filter with cutoff frequency and damping ratio.
     * This implementation uses the RBJ cookbook low-pass formula with pre-warping.
     *
     * @param cutoff_freq Cutoff frequency in Hz (-3dB point)
     * @param damping_ratio Damping ratio (zeta, controls Q = 1/(2*zeta))
     */
    void configure(type cutoff_freq, type damping_ratio) {
        const type fs = type(1.0) / this->get_sampling_time();
        // Compute normalized angular frequency (radians/sample)
        const type omega = type(2.0) * pi<type> * cutoff_freq / fs;
        const type cos_omega = std::cos(omega);
        const type sin_omega = std::sin(omega);
        // alpha controls the Q; here Q = 1/(2*damping_ratio)
        const type alpha = sin_omega / (type(2.0) * damping_ratio);

        // RBJ low-pass filter coefficients
        type b0 = (type(1.0) - cos_omega) / type(2.0);
        type b1 = type(1.0) - cos_omega;
        type b2 = (type(1.0) - cos_omega) / type(2.0);
        type a0 = type(1.0) + alpha;
        type a1 = -type(2.0) * cos_omega;
        type a2 = type(1.0) - alpha;

        // Normalize coefficients (set a0 = 1)
        this->_b0 = b0 / a0;
        this->_b1 = b1 / a0;
        this->_b2 = b2 / a0;
        this->_a1 = a1 / a0;
        this->_a2 = a2 / a0;
    }
};

/**
 * Second-order band-reject (notch) filter implementation.
 *
 * Mathematical Background:
 * ----------------------
 * The continuous-time transfer function is:
 *     H(s) = (s² + ω0²) / (s² + αs + ω0²)
 *
 * where:
 * - ω0 is the center frequency to reject
 * - α is the bandwidth parameter
 */
template <typename T>
class second_order_band_reject_filter : public second_order_filter<T> {
public:
    using type = fundamental_type_t<T>;

    second_order_band_reject_filter(type sampling_period) : second_order_filter<T>(sampling_period) {}

    /**
     * Configure filter with center frequency and bandwidth
     * @param center_freq Center frequency to reject (Hz)
     * @param bandwidth Bandwidth parameter (smaller means narrower notch)
     */
    void configure(type center_freq, type bandwidth) {
        // Sampling frequency and normalized omega:
        const type fs = type(1.0) / this->get_sampling_time();
        const type omega = type(2.0) * pi<type> * center_freq / fs;  // normalized angular frequency [rad/sample]
        const type quality_factor = center_freq / bandwidth;
        const type alpha = std::sin(omega) / (type(2.0) * quality_factor);

        // RBJ notch (band-reject) filter coefficients:
        type b0 = type(1.0);
        type b1 = -type(2.0) * std::cos(omega);
        type b2 = type(1.0);
        type a0 = type(1.0) + alpha;
        type a1 = -type(2.0) * std::cos(omega);
        type a2 = type(1.0) - alpha;

        // Normalize coefficients (set a0 = 1)
        this->_b0 = b0 / a0;
        this->_b1 = b1 / a0;
        this->_b2 = b2 / a0;
        this->_a1 = a1 / a0;
        this->_a2 = a2 / a0;
    }
};

/**
 * Second-order high-pass filter implementation.
 *
 * Mathematical Background:
 * ----------------------
 * The continuous-time transfer function is:
 *     H(s) = s² / (s² + 2ζωc*s + ωc²)
 *
 * where:
 * - ωc is the cutoff frequency in radians/second
 * - ζ is the damping ratio
 */
template <typename T>
class second_order_high_pass_filter : public second_order_filter<T> {
public:
    using type = fundamental_type_t<T>;

    second_order_high_pass_filter(type sampling_period) : second_order_filter<T>(sampling_period) {}

    /**
     * Configure filter with cutoff frequency and damping ratio.
     * This implementation uses the RBJ cookbook high-pass formula.
     *
     * @param cutoff_freq Cutoff frequency in Hz (-3dB point)
     * @param damping_ratio Damping ratio (zeta, controls Q = 1/(2*zeta))
     */
    void configure(type cutoff_freq, type damping_ratio) {
        const type fs = type(1.0) / this->get_sampling_time();
        // Compute normalized angular frequency (radians/sample)
        const type omega = type(2.0) * pi<type> * cutoff_freq / fs;
        const type cos_omega = std::cos(omega);
        const type sin_omega = std::sin(omega);
        // α controls the Q; here Q = 1/(2*damping_ratio)
        const type alpha = sin_omega / (type(2.0) * damping_ratio);

        // RBJ high-pass filter coefficients
        type b0 = (type(1.0) + cos_omega) / type(2.0);
        type b1 = -(type(1.0) + cos_omega);
        type b2 = (type(1.0) + cos_omega) / type(2.0);
        type a0 = type(1.0) + alpha;
        type a1 = -type(2.0) * cos_omega;
        type a2 = type(1.0) - alpha;

        // Normalize coefficients (set a0 = 1)
        this->_b0 = b0 / a0;
        this->_b1 = b1 / a0;
        this->_b2 = b2 / a0;
        this->_a1 = a1 / a0;
        this->_a2 = a2 / a0;
    }
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_SECOND_ORDER_FILTER_HPP