#include "sfc/grid_side_hardware_mock.hpp"

namespace yeet::application::sfc::mock {

grid_side_hardware_mock::grid_side_hardware_mock(duration_t sampling_period)
    : _v_ab_adc(sampling_period), _v_bc_adc(sampling_period), _i_a_adc(sampling_period), _i_c_adc(sampling_period)
{
}

void grid_side_hardware_mock::configure_voltage_adc(
    real_t gain, frequency_t cutoff_freq_hz, int bits, real_t full_scale_voltage)
{
    // Store conversion parameters
    _voltage_bits = bits;
    _voltage_full_scale = full_scale_voltage;

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
    // Store conversion parameters
    _current_bits = bits;
    _current_full_scale = full_scale_current;

    _i_a_adc.configure_gain(gain);
    _i_a_adc.configure_filter(cutoff_freq_hz);
    _i_a_adc.configure_quantization(bits, full_scale_current);

    _i_c_adc.configure_gain(gain);
    _i_c_adc.configure_filter(cutoff_freq_hz);
    _i_c_adc.configure_quantization(bits, full_scale_current);
}

void grid_side_hardware_mock::update(real_t v_ab, real_t v_bc, real_t i_a, real_t i_c) noexcept
{
    _v_ab_adc.update(v_ab);
    _v_bc_adc.update(v_bc);
    _i_a_adc.update(i_a);
    _i_c_adc.update(i_c);
}

void grid_side_hardware_mock::reset()
{
    _v_ab_adc.reset();
    _v_bc_adc.reset();
    _i_a_adc.reset();
    _i_c_adc.reset();
}

yeet::control::adc_value_t grid_side_hardware_mock::get_v_ab_counts() const noexcept
{
    return yeet::control::adc_value_t(_voltage_bits, _voltage_full_scale, _v_ab_adc.get_output());
}

yeet::control::adc_value_t grid_side_hardware_mock::get_v_bc_counts() const noexcept
{
    return yeet::control::adc_value_t(_voltage_bits, _voltage_full_scale, _v_bc_adc.get_output());
}

yeet::control::adc_value_t grid_side_hardware_mock::get_i_a_counts() const noexcept
{
    return yeet::control::adc_value_t(_current_bits, _current_full_scale, _i_a_adc.get_output());
}

yeet::control::adc_value_t grid_side_hardware_mock::get_i_c_counts() const noexcept
{
    return yeet::control::adc_value_t(_current_bits, _current_full_scale, _i_c_adc.get_output());
}

}  // namespace yeet::application::sfc::mock