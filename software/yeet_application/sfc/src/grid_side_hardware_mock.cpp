#include "sfc/grid_side_hardware_mock.hpp"

namespace yeet::application::sfc::mock {

grid_side_hardware_mock::grid_side_hardware_mock(duration_t sampling_period)
    : _v_ab_adc(sampling_period), _v_bc_adc(sampling_period), _i_a_adc(sampling_period), _i_c_adc(sampling_period)
{
}

void grid_side_hardware_mock::configure_voltage_adc(
    real_t gain, frequency_t cutoff_freq_hz, int bits, real_t full_scale_voltage)
{
    _voltage_bits = bits;
    _voltage_full_scale = full_scale_voltage;
    _voltage_converter.configure(bits, full_scale_voltage);

    _v_ab_adc.configure_gain(gain);
    _v_ab_adc.configure_filter(cutoff_freq_hz);
    _v_ab_adc.configure_quantization(bits, full_scale_voltage);

    _v_bc_adc.configure_gain(gain);
    _v_bc_adc.configure_filter(cutoff_freq_hz);
    _v_bc_adc.configure_quantization(bits, full_scale_voltage);
}

void grid_side_hardware_mock::configure_current_adc(
    real_t gain, frequency_t cutoff_freq_hz, int bits, real_t full_scale_current)
{
    _current_bits = bits;
    _current_full_scale = full_scale_current;
    _current_converter.configure(bits, full_scale_current);

    _i_a_adc.configure_gain(gain);
    _i_a_adc.configure_filter(cutoff_freq_hz);
    _i_a_adc.configure_quantization(bits, full_scale_current);

    _i_c_adc.configure_gain(gain);
    _i_c_adc.configure_filter(cutoff_freq_hz);
    _i_c_adc.configure_quantization(bits, full_scale_current);
}

void grid_side_hardware_mock::update(real_t v_ab, real_t v_bc, real_t i_a, real_t i_c) noexcept
{
    // Apply transformer ratios: convert primary values to secondary before ADC chain
    const real_t v_ab_sec = v_ab / _pt_ratio;  // uses precomputed reciprocal
    const real_t v_bc_sec = v_bc / _pt_ratio;
    const real_t i_a_sec = i_a / _ct_ratio;
    const real_t i_c_sec = i_c / _ct_ratio;

    _v_ab_adc.update(v_ab_sec);
    _v_bc_adc.update(v_bc_sec);
    _i_a_adc.update(i_a_sec);
    _i_c_adc.update(i_c_sec);
}

void grid_side_hardware_mock::reset()
{
    _v_ab_adc.reset();
    _v_bc_adc.reset();
    _i_a_adc.reset();
    _i_c_adc.reset();
}

// Getters now inline in header returning adc_counts only; conversion handled elsewhere.

void grid_side_hardware_mock::configure_transformer_ratios(real_t pt_ratio, real_t ct_ratio) noexcept
{
    // Basic validation (avoid divide-by-zero later). If invalid, clamp to 1.0.
    _pt_ratio = yeet::precomputed_divisor<real_t>{(pt_ratio > real_t{0}) ? pt_ratio : real_t{1.0}};
    _ct_ratio = yeet::precomputed_divisor<real_t>{(ct_ratio > real_t{0}) ? ct_ratio : real_t{1.0}};
}

void grid_side_hardware_mock::configure_voltage_transformer(
    real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept
{
    if (secondary_ll_voltage > real_t{0}) {
        _pt_ratio = yeet::precomputed_divisor<real_t>{primary_ll_voltage / secondary_ll_voltage};
    }
}

void grid_side_hardware_mock::configure_current_transformer(real_t primary_current, real_t secondary_current) noexcept
{
    if (secondary_current > real_t{0}) {
        _ct_ratio = yeet::precomputed_divisor<real_t>{primary_current / secondary_current};
    }
}

void grid_side_hardware_mock::configure_transformers(
    real_t primary_ll_voltage, real_t secondary_ll_voltage, real_t primary_current, real_t secondary_current) noexcept
{
    configure_voltage_transformer(primary_ll_voltage, secondary_ll_voltage);
    configure_current_transformer(primary_current, secondary_current);
}

}  // namespace yeet::application::sfc::mock