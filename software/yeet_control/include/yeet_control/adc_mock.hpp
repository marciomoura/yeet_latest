#pragma once

#include <cstdint>

#include "first_order_low_pass_filter.hpp"
#include "units.hpp"  // provides real_t alias used in interface

namespace yeet::control {
namespace mock {

/**
 * @class adc_mock
 * @brief Mocks an ADC signal chain including an amplifier, low-pass filter, and quantizer.
 *
 * This class simulates a typical analog-to-digital converter input stage.
 * The signal processing chain is as follows:
 * 1. Input signal is amplified.
 * 2. Amplified signal is filtered by a first-order low-pass filter.
 * 3. Filtered signal is quantized to a specified bit depth.
 */
class adc_mock {
public:
    /**
     * @brief Constructs the ADC mock.
     * @param sampling_period The sampling period (Ts) in seconds for the filter and ADC.
     */
    explicit adc_mock(real_t sampling_period);

    /**
     * @brief Configures the gain of the input amplifier stage.
     * @param gain The amplification factor.
     */
    void configure_gain(real_t gain);

    /**
     * @brief Configures the cutoff frequency of the anti-aliasing low-pass filter.
     * @param cutoff_freq_hz The filter's cutoff frequency in Hertz.
     */
    void configure_filter(real_t cutoff_freq_hz);

    /**
     * @brief Configures the quantization parameters (generic analog quantity).
     * @param bits The number of bits for the ADC (e.g., 12, 16).
     * @param full_scale_value The maximum magnitude of the ADC's bipolar input range. Range assumed [-V,+V] where V =
     * full_scale_value.
     */
    void configure_quantization(int bits, real_t full_scale_value);

    /**
     * @brief Processes a new input voltage sample through the mock ADC chain.
     *
     * The input is amplified, filtered, and then quantized. The result is
     * available via get_output(). This method is noexcept and real-time safe.
     *
     * @param input_voltage The raw analog input voltage.
     */
    void update(real_t input_value) noexcept;

    /**
     * @brief Resets the internal state of the component.
     *
     * Resets the low-pass filter's state and sets the output to zero.
     */
    void reset();

    /**
     * @brief Gets the latest quantized output value from the ADC.
     * @return The quantized output as a signed 32-bit integer.
     */
    std::int32_t get_output() const noexcept;  ///< Latest quantized counts (two's complement style range)

    real_t get_filtered() const noexcept { return _last_filtered; }  ///< Last filtered analog value pre-quantization
    int get_bits() const noexcept { return _bits; }
    real_t get_full_scale() const noexcept { return _full_scale_value; }

    // Compile-time guard ensuring real_t is a floating type (uses units.hpp definition).
    // Ensure units infrastructure is available (quantity defined in units.hpp)
    static_assert(std::is_floating_point_v<real_t>, "real_t must be a floating point type");

private:
    real_t _gain{1.0};
    first_order_low_pass_filter<real_t> _filter;
    int _bits{12};
    real_t _full_scale_value{1.0};
    std::int32_t _output{0};
    real_t _last_filtered{0.0};
};

}  // namespace mock
}  // namespace yeet::control
