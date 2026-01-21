#include "yeet_control/hardware_filter_compensator.hpp"

#include <cmath>

namespace yeet::control {

void hardware_filter_compensator::configure(frequency_t cutoff_f1, frequency_t cutoff_f2, int num_filters)
{
    // Reset to safe defaults
    _num_filters = 0;
    _cutoff_f1_divisor = precomputed_divisor<frequency_t>(frequency_t{0.0f});
    _cutoff_f2_divisor = precomputed_divisor<frequency_t>(frequency_t{0.0f});

    if (num_filters <= 0) {
        return;
    }

    // Configure Stage 1 if frequency is valid and at least one filter is requested
    if (cutoff_f1.value() > 0.0f) {
        _cutoff_f1_divisor = precomputed_divisor<frequency_t>(cutoff_f1);
        _num_filters = 1;
    }

    // Configure Stage 2 if frequency is valid and two filters are requested
    if (num_filters >= 2 && cutoff_f2.value() > 0.0f) {
        _cutoff_f2_divisor = precomputed_divisor<frequency_t>(cutoff_f2);
        // We set _num_filters to 2. If Stage 1 was invalid, update() will skip it but run Stage 2.
        _num_filters = 2;
    }
}

void hardware_filter_compensator::reset()
{
    _cutoff_f1_divisor = precomputed_divisor<frequency_t>(frequency_t{0.0f});
    _cutoff_f2_divisor = precomputed_divisor<frequency_t>(frequency_t{0.0f});
    _num_filters = 0;
}

}  // namespace yeet::control
