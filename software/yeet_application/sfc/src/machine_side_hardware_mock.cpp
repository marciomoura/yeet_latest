#include "sfc/machine_side_hardware_mock.hpp"

namespace yeet::application::sfc::mock {

machine_side_hardware_mock::machine_side_hardware_mock(duration_t sampling_period)
    : _v_ab_adc(sampling_period), _v_bc_adc(sampling_period)
{
}

void machine_side_hardware_mock::configure_voltage_adc(
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

void machine_side_hardware_mock::update(real_t v_ab, real_t v_bc) noexcept
{
    const real_t v_ab_sec = v_ab / _pt_ratio;  // fast multiply via reciprocal
    const real_t v_bc_sec = v_bc / _pt_ratio;
    _v_ab_adc.update(v_ab_sec);
    _v_bc_adc.update(v_bc_sec);
}

void machine_side_hardware_mock::reset()
{
    _v_ab_adc.reset();
    _v_bc_adc.reset();
}

void machine_side_hardware_mock::configure_transformer_ratio(real_t pt_ratio) noexcept
{
    _pt_ratio = yeet::precomputed_divisor<real_t>{(pt_ratio > real_t{0}) ? pt_ratio : real_t{1.0}};
}

void machine_side_hardware_mock::configure_voltage_transformer(
    real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept
{
    if (secondary_ll_voltage > real_t{0}) {
        _pt_ratio = yeet::precomputed_divisor<real_t>{primary_ll_voltage / secondary_ll_voltage};
    }
}

}  // namespace yeet::application::sfc::mock