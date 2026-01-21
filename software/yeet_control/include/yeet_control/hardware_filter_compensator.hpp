#pragma once

#include "precomputed_divisor.hpp"
#include "reference_frame.hpp"
#include "units.hpp"

namespace yeet::control {

/**
 * @brief Compensates for up to two series-connected first-order hardware low-pass filters.
 *
 * This component calculates the inverse transfer function of a first-order LPF
 * at runtime to compensate for phase lag and gain attenuation.
 *
 * The compensation formula for a single filter is:
 *   V_comp = G * R(phi) * V
 * where phi = lead angle = arctan(f/fc)
 * and G = gain = sqrt(1 + (f/fc)^2) -> Saturated to 1.0 to avoid noise amplification.
 */
class hardware_filter_compensator {
public:
    hardware_filter_compensator() = default;

    /**
     * @brief Configures the hardware filter cut-off frequencies.
     * @param cutoff_f1 The cut-off frequency of the first filter stage.
     * @param cutoff_f2 The cut-off frequency of the second filter stage (optional).
     * @param num_filters The number of active filter stages to compensate (0, 1, or 2).
     */
    void configure(frequency_t cutoff_f1, frequency_t cutoff_f2 = frequency_t{0.0f}, int num_filters = 1);

    /**
     * @brief Resets the compensation state.
     */
    void reset();

    /**
     * @brief Updates the compensation based on the current frequency.
     * @param input The three-phase input signal to be compensated.
     * @param frequency The current estimated frequency of the system.
     * @return The compensated three-phase signal.
     */
    template <typename T>
    abc<T> update(const abc<T>& input, frequency_t frequency)
    {
        if (_num_filters <= 0 || frequency.value() <= 0.0f) {
            return input;
        }

        angle_wrapped total_phase = 0.0f;
        real_t total_gain = 1.0f;

        // Stage 1: Always run if at least one filter is requested and configured
        if (_num_filters >= 1 && _cutoff_f1_divisor.value().value() > 0.0f) {
            const real_t ratio = frequency / _cutoff_f1_divisor;
            total_phase += angle_wrapped::from_radians(angle_t{std::atan(ratio)});
            total_gain *= std::sqrt(1.0f + ratio * ratio);
        }

        // Stage 2: Only run if two filters are requested and configured
        if (_num_filters >= 2 && _cutoff_f2_divisor.value().value() > 0.0f) {
            const real_t ratio = frequency / _cutoff_f2_divisor;
            total_phase += angle_wrapped::from_radians(angle_t{std::atan(ratio)});
            total_gain *= std::sqrt(1.0f + ratio * ratio);
        }

        // Saturate gain to 1.0 as requested to avoid noise amplification
        const real_t gain_comp = std::min(total_gain, 1.0f);

        // Convert to alpha-beta for rotation
        const alphabeta<T> ab = gain_comp * input.to_alphabeta();

        return ab.rotate_counter_clockwise(angle_wrapped{total_phase}).to_abc();
    }

private:
    int _num_filters{0};
    precomputed_divisor<frequency_t> _cutoff_f1_divisor;
    precomputed_divisor<frequency_t> _cutoff_f2_divisor;
};

}  // namespace yeet::control
