#include "yeet_control/adc_mock.hpp"

#include <algorithm>
#include <cmath>

namespace yeet::control::mock {

adc_mock::adc_mock(real_t sampling_period) : _filter(sampling_period) {}

void adc_mock::configure_gain(real_t gain) { _gain = gain; }

void adc_mock::configure_filter(real_t cutoff_freq_hz) { _filter.configure(cutoff_freq_hz); }

void adc_mock::configure_quantization(int bits, real_t full_scale_voltage)
{
    _bits = bits;
    _full_scale_voltage = full_scale_voltage;
}

void adc_mock::update(real_t input_voltage) noexcept
{
    // 1. Amplify the signal
    const real_t amplified_signal = input_voltage * _gain;

    // 2. Filter the signal
    _filter.update(amplified_signal);
    const real_t filtered_signal = _filter.get_output();

    // 3. Quantize the signal
    const real_t max_val = _full_scale_voltage;
    const real_t min_val = -_full_scale_voltage;

    // Clamp the signal to the ADC's full-scale range
    const real_t clamped_signal = std::clamp(filtered_signal, min_val, max_val);

    // Calculate the maximum integer value for the given bit depth (for a signed integer)
    const std::int32_t max_adc_count = (1 << (_bits - 1)) - 1;

    // Scale the clamped signal to the integer range and round to the nearest integer
    _output = static_cast<std::int32_t>(std::round((clamped_signal / _full_scale_voltage) * max_adc_count));
}

void adc_mock::reset()
{
    _filter.reset();
    _output = 0;
}

std::int32_t adc_mock::get_output() const noexcept { return _output; }

}  // namespace yeet::control::mock