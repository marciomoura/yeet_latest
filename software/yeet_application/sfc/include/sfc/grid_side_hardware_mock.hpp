#pragma once

#include "yeet_control/adc_mock.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/precomputed_divisor.hpp"
#include "yeet_control/units.hpp"


namespace yeet::application::sfc::mock {

/**
 * @class grid_side_hardware_mock
 * @brief Mocks the measurement channels for grid-side voltages and currents.
 *
 * This class uses four instances of adc_mock to simulate the sampling and
 * quantization of two line-to-line voltages (Vab, Vbc) and two line
 * currents (Ia, Ic).
 */
class grid_side_hardware_mock {
public:
    /**
     * @brief Constructs the grid-side measurements mock.
     * @param sampling_period The sampling period (Ts) in seconds for all ADC channels.
     */
    explicit grid_side_hardware_mock(duration_t sampling_period);

    /**
     * @brief Configures all voltage ADC channels with the same parameters.
     * @param gain The amplification factor.
     * @param cutoff_freq_hz The anti-aliasing filter's cutoff frequency in Hertz.
     * @param bits The number of bits for the ADC.
     * @param full_scale_voltage The ADC's full-scale voltage range.
     */
    void configure_voltage_adc(real_t gain, frequency_t cutoff_freq_hz, int bits, real_t full_scale_voltage);

    /**
     * @brief Configures all current ADC channels with the same parameters.
     * @param gain The amplification factor.
     * @param cutoff_freq_hz The anti-aliasing filter's cutoff frequency in Hertz.
     * @param bits The number of bits for the ADC.
     * @param full_scale_current The ADC's full-scale current range.
     */
    void configure_current_adc(real_t gain, frequency_t cutoff_freq_hz, int bits, real_t full_scale_current);

    /**
     * @brief Configures potential (PT) and current (CT) transformer ratios.
     *
     * Ratios are defined as Primary / Secondary. For example, a PT of 6000 V / 100 V
     * should be passed as pt_ratio = 6000.0/100.0 = 60. A CT of 1000 A / 5 A
     * becomes ct_ratio = 1000.0/5.0 = 200. These ratios are applied in the update()
     * path so that the analog value presented to the transducer/ADC chain is:
     *   V_secondary = V_primary / pt_ratio
     *   I_secondary = I_primary / ct_ratio
     *
     * Default ratios are 1.0 (no scaling) to preserve existing behavior.
     *
     * @param pt_ratio Primary-to-secondary voltage ratio (Primary_V / Secondary_V)
     * @param ct_ratio Primary-to-secondary current ratio (Primary_A / Secondary_A)
     */
    void configure_transformer_ratios(real_t pt_ratio, real_t ct_ratio) noexcept;

    /**
     * @brief Configure voltage (PT) transformer using primary & secondary line-to-line voltages.
     *
     * Computes pt_ratio = primary_ll_voltage / secondary_ll_voltage and stores it.
     * If secondary_ll_voltage <= 0 the ratio is left at 1.0 (no scaling).
     *
     * @param primary_ll_voltage  Primary line-to-line voltage (V)
     * @param secondary_ll_voltage Secondary line-to-line voltage presented by the PT (V)
     */
    void configure_voltage_transformer(real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept;

    /**
     * @brief Configure current (CT) transformer using primary & secondary currents.
     *
     * Computes ct_ratio = primary_current / secondary_current and stores it.
     * If secondary_current <= 0 the ratio is left at 1.0 (no scaling).
     *
     * @param primary_current  Primary current (A)
     * @param secondary_current Secondary current provided by CT (A)
     */
    void configure_current_transformer(real_t primary_current, real_t secondary_current) noexcept;

    /**
     * @brief Convenience method configuring both PT and CT from primary/secondary values.
     */
    void configure_transformers(real_t primary_ll_voltage,
        real_t secondary_ll_voltage,
        real_t primary_current,
        real_t secondary_current) noexcept;

    /**
     * @brief Processes new analog input values through the mock ADC chains.
     * @param v_ab The raw analog line-to-line voltage Vab.
     * @param v_bc The raw analog line-to-line voltage Vbc.
     * @param i_a The raw analog line current Ia.
     * @param i_c The raw analog line current Ic.
     */
    void update(real_t v_ab, real_t v_bc, real_t i_a, real_t i_c) noexcept;

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
    /** @return Raw ADC counts for current Ia. */
    yeet::control::adc_counts get_i_a_counts() const noexcept
    {
        return yeet::control::adc_counts{_i_a_adc.get_output()};
    }
    /** @return Raw ADC counts for current Ic. */
    yeet::control::adc_counts get_i_c_counts() const noexcept
    {
        return yeet::control::adc_counts{_i_c_adc.get_output()};
    }

private:
    yeet::control::mock::adc_mock _v_ab_adc;
    yeet::control::mock::adc_mock _v_bc_adc;
    yeet::control::mock::adc_mock _i_a_adc;
    yeet::control::mock::adc_mock _i_c_adc;

    // Transformer ratios (Primary / Secondary). Defaults: 1.0 -> identity.
    // Stored as divisors so primary -> secondary uses fast multiply via reciprocal
    yeet::precomputed_divisor<real_t> _pt_ratio{real_t{1.0}};  // V_secondary = V_primary / pt_ratio
    yeet::precomputed_divisor<real_t> _ct_ratio{real_t{1.0}};  // I_secondary = I_primary / ct_ratio

    // Embedded converters (configured alongside adc_mocks)
    yeet::control::adc_converter _voltage_converter{};  // bits/full-scale for voltage channels
    yeet::control::adc_converter _current_converter{};  // bits/full-scale for current channels
    int _voltage_bits{12};
    real_t _voltage_full_scale{1.0};
    int _current_bits{12};
    real_t _current_full_scale{1.0};

public:
    // Access to converters so higher layers can translate counts -> real (e.g., measurement classes)
    const yeet::control::adc_converter& get_voltage_converter() const noexcept { return _voltage_converter; }
    const yeet::control::adc_converter& get_current_converter() const noexcept { return _current_converter; }
};

}  // namespace yeet::application::sfc::mock