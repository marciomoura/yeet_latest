#pragma once

#include "yeet_control/adc_mock.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/precomputed_divisor.hpp"
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
     * @brief Configures potential transformer (PT) ratio for machine-side voltages.
     *
     * Ratio is defined Primary / Secondary. Example: 3300 V / 100 V => pt_ratio = 33.
     * The analog voltage seen by ADC chain is V_secondary = V_primary / pt_ratio.
     * Default is 1.0 (no scaling).
     *
     * @param pt_ratio Primary-to-secondary voltage ratio (Primary_V / Secondary_V)
     */
    void configure_transformer_ratio(real_t pt_ratio) noexcept;

    /**
     * @brief Configure PT using primary & secondary line-to-line voltages.
     * pt_ratio = primary_ll_voltage / secondary_ll_voltage.
     * If secondary_ll_voltage <= 0 the ratio remains unchanged.
     */
    void configure_voltage_transformer(real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept;

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

    /** @return Raw ADC counts for voltage Vab. */
    yeet::control::adc_counts get_v_ab_counts() const noexcept
    {
        return yeet::control::adc_counts{_v_ab_adc.get_output()};
    }
    /** @return Raw ADC counts for voltage Vbc. */
    yeet::control::adc_counts get_v_bc_counts() const noexcept
    {
        return yeet::control::adc_counts{_v_bc_adc.get_output()};
    }

private:
    yeet::control::mock::adc_mock _v_ab_adc;
    yeet::control::mock::adc_mock _v_bc_adc;

    // Embedded converter for voltage channels
    yeet::control::adc_converter _voltage_converter{};
    int _voltage_bits{12};
    real_t _voltage_full_scale{1.0};
    yeet::precomputed_divisor<real_t> _pt_ratio{real_t{1.0}};  // V_secondary = V_primary / pt_ratio

public:
    const yeet::control::adc_converter& get_voltage_converter() const noexcept { return _voltage_converter; }
};

}  // namespace yeet::application::sfc::mock