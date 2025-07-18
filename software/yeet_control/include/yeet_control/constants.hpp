#ifndef YEET_CONTROL_CONSTANTS_HPP
#define YEET_CONTROL_CONSTANTS_HPP

#include "types.hpp"

namespace yeet {

static constexpr real_t pi = real_t(3.1415926535897932385);

static constexpr real_t two_pi = real_t(2.0) * pi;

constexpr real_t deg_to_rad(real_t degrees) { return degrees * pi / real_t(180.0); }

template <typename real_t>
constexpr real_t rad_to_deg(real_t radians) {
    return radians * real_t(180.0) / pi;
}

// --- RMS/Peak Conversions ---
constexpr real_t rms_to_peak_factor = 1.41421356237;  // std::sqrt(2.0);
constexpr real_t peak_to_rms_factor = 1.0 / rms_to_peak_factor;

// --- Phase/Line Conversions ---
constexpr real_t phase_to_line_rms_factor = 1.73205080757;  // std::sqrt(3.0);
constexpr real_t line_to_phase_rms_factor = 1.0 / phase_to_line_rms_factor;

// --- Clarke Transform ---
constexpr real_t two_thirds = 2.0 / 3.0;
constexpr real_t sqrt_two_thirds = 0.81649658092;  // std::sqrt(2.0 / 3.0);

constexpr real_t sqrt_2 = 1.41421356237;        // std::sqrt(2.0);
constexpr real_t sqrt_3 = 1.73205080757;        // std::sqrt(3.0);
constexpr real_t sqrt_3_over_2 = sqrt_3 / 2.0;  // std::sqrt(3.0)/2.0;

}  // namespace yeet

#endif  // YEET_CONTROL_CONSTANTS_HPP