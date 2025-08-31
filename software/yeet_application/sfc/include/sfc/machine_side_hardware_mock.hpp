#pragma once

#include "yeet_control/adc_mock.hpp"
#include "yeet_control/adc_value_t.hpp"
#include "yeet_control/units.hpp"


namespace yeet::application::sfc::mock {

/**
 * @class machine_side_hardware_mock
 * @brief Mocks the measurement channels for machine-side voltages.
 *
 * This class uses two instances of adc_mock to simulate the sampling and
 * quantization of two line-to-line voltages (Vab, Vbc).
 */
class machine_side_hardware_mock {
public:
    /**
     * @brief Constructs the machine-side measurements mock.
     * @param sampling_period The sampling period (Ts) in seconds for all ADC channels.
     */
    explicit machine_side_hardware_mock(duration_t sampling_period);

    /**
     * @brief Configures all voltage ADC channels with the same parameters.
     * @param gain The amplification factor.
     * @param cutoff_freq_hz The anti-aliasing filter's cutoff frequency in Hertz.
     * @param bits The number of bits for the ADC.
     * @param full_scale_voltage The ADC's full-scale voltage range.
     */
    void configure_voltage_adc(real_t gain, frequency_t cutoff_freq_hz, int bits, real_t full_scale_voltage);

    /**
     * @brief Processes new analog input values through the mock ADC chains.
     * @param v_ab The raw analog line-to-line voltage Vab.
     * @param v_bc The raw analog line-to-line voltage Vbc.
     */
    void update(real_t v_ab, real_t v_bc) noexcept;

    /**
     * @brief Resets the internal state of all ADC channels.
     */
    void reset();

    /** @return The latest quantized output for voltage Vab. */
    yeet::control::adc_value_t get_v_ab_counts() const noexcept;

    /** @return The latest quantized output for voltage Vbc. */
    yeet::control::adc_value_t get_v_bc_counts() const noexcept;

private:
    yeet::control::mock::adc_mock _v_ab_adc;
    yeet::control::mock::adc_mock _v_bc_adc;

    // Conversion parameters for creating adc_value_t instances
    int _voltage_bits{12};
    real_t _voltage_full_scale{1.0};
};

}  // namespace yeet::application::sfc::mock