#include <cassert>
#include <cmath>

#include "yeet_control/adc_value.hpp"

namespace yeet::control {

adc_value_t::adc_value_t(int bits, real_t full_scale_value, std::int32_t counts) : _counts(counts)
{
    _calculate_factors(bits, full_scale_value);
}

adc_value_t adc_value_t::from_real(int bits, real_t full_scale_value, real_t real_value)
{
    adc_value_t result(bits, full_scale_value);
    result.set_real(real_value);
    return result;
}

void adc_value_t::set_counts(std::int32_t counts) noexcept { _counts = counts; }

void adc_value_t::set_real(real_t real_value) noexcept
{
    // Convert real value to counts and clamp to valid range
    const auto counts_float = real_value * _real_to_counts_factor;
    _counts = static_cast<std::int32_t>(std::round(counts_float));

    // Clamp to valid ADC range
    if (_counts > _max_counts) {
        _counts = _max_counts;
    }
    else if (_counts < -_max_counts - 1) {
        _counts = -_max_counts - 1;
    }
}

std::int32_t adc_value_t::get_counts() const noexcept { return _counts; }

real_t adc_value_t::to_real() const noexcept { return static_cast<real_t>(_counts) * _counts_to_real_factor; }

adc_value_t::operator real_t() const noexcept { return to_real(); }

adc_value_t& adc_value_t::operator=(std::int32_t counts) noexcept
{
    set_counts(counts);
    return *this;
}

adc_value_t& adc_value_t::operator=(real_t real_value) noexcept
{
    set_real(real_value);
    return *this;
}

void adc_value_t::_calculate_factors(int bits, real_t full_scale_value)
{
    assert(bits > 1);
    assert(full_scale_value > 0.0);

    // Calculate the maximum integer value for a signed integer of 'bits' resolution.
    _max_counts = (1 << (bits - 1)) - 1;

    if (_max_counts > 0) {
        _counts_to_real_factor = full_scale_value / static_cast<real_t>(_max_counts);
        _real_to_counts_factor = static_cast<real_t>(_max_counts) / full_scale_value;
    }
}

}  // namespace yeet::control