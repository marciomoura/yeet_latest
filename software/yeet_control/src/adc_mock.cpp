#include "yeet_control/adc_mock.hpp"

#include <algorithm>
#include <cmath>

namespace yeet::control::mock {

adc_mock::adc_mock(real_t sampling_period) : _filter(sampling_period) {}

void adc_mock::configure_gain(real_t gain) { _gain = gain; }

void adc_mock::configure_filter(real_t cutoff_freq_hz) { _filter.configure(cutoff_freq_hz); }

void adc_mock::configure_quantization(int bits, real_t full_scale_value)
{
    _bits = bits;
    _full_scale_value = full_scale_value;
}

void adc_mock::update(real_t input_value) noexcept
{
    const real_t amplified = input_value * _gain;  // 1. apply gain
    _filter.update(amplified);                     // 2. low-pass filter
    _last_filtered = _filter.get_output();

    const real_t max_val = _full_scale_value;
    const real_t min_val = -_full_scale_value;
    const real_t clamped = std::clamp(_last_filtered, min_val, max_val);  // 3. clamp

    const std::int32_t max_counts = (1 << (_bits - 1)) - 1;  // symmetric bipolar
    _output = static_cast<std::int32_t>(std::llround((clamped / _full_scale_value) * max_counts));
}

void adc_mock::reset()
{
    _filter.reset();
    _output = 0;
    _last_filtered = 0.0;
}

std::int32_t adc_mock::get_output() const noexcept { return _output; }

}  // namespace yeet::control::mock