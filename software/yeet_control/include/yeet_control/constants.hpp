#ifndef YEET_CONTROL_CONSTANTS_HPP
#define YEET_CONTROL_CONSTANTS_HPP

#include "types.hpp"
#include "units.hpp"

namespace yeet {

static constexpr double pi_double = double(3.1415926535897932385);

static constexpr real_t pi = real_t(pi_double);

static constexpr real_t two_pi = real_t(2.0 * pi_double);

constexpr angle_t deg_to_rad(real_t degrees) { return angle_t{degrees * pi / real_t(180.0)}; }

constexpr real_t rad_to_deg(angle_t radians) { return radians.value() * real_t(180.0) / pi; }

constexpr angular_frequency_t hz_to_rad_per_sec(real_t hertz) { return angular_frequency_t{hertz * two_pi}; }

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

// Define a ms constant for convenience
constexpr duration_t ms(real_t milliseconds) { return duration_t(milliseconds * 1e-3); }
// Define a us constant for convenience
constexpr duration_t us(real_t microseconds) { return duration_t(microseconds * 1e-6); }
// Define a ns constant for convenience
constexpr duration_t ns(real_t nanoseconds) { return duration_t(nanoseconds * 1e-9); }

}  // namespace yeet

#endif  // YEET_CONTROL_CONSTANTS_HPP