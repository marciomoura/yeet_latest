#include "yeet_control/hardware_filter_compensator.hpp"

#include <cmath>

namespace yeet::control {

void hardware_filter_compensator::configure_as_first_order_lpf(frequency_t cutoff_frequency, frequency_t max_frequency)
{
    if (cutoff_frequency.value() <= 0.0f) {
        // Effective bypass if cutoff is zero or negative
        reset();
        return;
    }

    std::array<real_t, num_points> frequencies;
    std::array<real_t, num_points> gains;
    std::array<real_t, num_points> phases;

    const real_t fc = cutoff_frequency.value();
    const real_t f_max = max_frequency.value();
    const real_t f_step = f_max / static_cast<real_t>(num_points - 1);

    for (size_t i = 0; i < num_points; ++i) {
        const real_t f = static_cast<real_t>(i) * f_step;
        const real_t ratio = f / fc;

        frequencies[i] = f;
        // Inverse of 1/(1+js/wc) is (1+js/wc)
        // Gain: sqrt(1 + (f/fc)^2)
        // Phase: atan(f/fc)
        gains[i] = std::sqrt(1.0f + ratio * ratio);
        phases[i] = std::atan(ratio);
    }

    _gain_lut.configure(frequencies, gains);
    _phase_lut.configure(frequencies, phases);
}

void hardware_filter_compensator::reset()
{
    std::array<real_t, num_points> frequencies;
    std::array<real_t, num_points> ones;
    std::array<real_t, num_points> zeros;

    frequencies.fill(0.0f);
    ones.fill(1.0f);
    zeros.fill(0.0f);

    // Ensure axis is sorted even for reset
    for (size_t i = 0; i < num_points; ++i) {
        frequencies[i] = static_cast<real_t>(i);
    }

    _gain_lut.configure(frequencies, ones);
    _phase_lut.configure(frequencies, zeros);
}

}  // namespace yeet::control
