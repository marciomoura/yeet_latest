#ifndef YEET_CONTROL_CONSTANTS_HPP
#define YEET_CONTROL_CONSTANTS_HPP

namespace yeet::control {

template <typename T>
static constexpr T pi = T(3.1415926535897932385);

template <typename T>
static constexpr T two_pi = T(2.0) * pi<T>;

// --- RMS/Peak Conversions ---
constexpr double rms_to_peak_factor = 1.41421356237;  // std::sqrt(2.0);
constexpr double peak_to_rms_factor = 1.0 / rms_to_peak_factor;

// --- Phase/Line Conversions ---
constexpr double phase_to_line_rms_factor = 1.73205080757;  // std::sqrt(3.0);
constexpr double line_to_phase_rms_factor = 1.0 / phase_to_line_rms_factor;

// --- Clarke Transform ---
constexpr double two_thirds = 2.0 / 3.0;
constexpr double sqrt_two_thirds = 0.81649658092;  // std::sqrt(2.0 / 3.0);

constexpr double sqrt_2 = 1.41421356237;        // std::sqrt(2.0);
constexpr double sqrt_3 = 1.73205080757;        // std::sqrt(3.0);
constexpr double sqrt_3_over_2 = sqrt_3 / 2.0;  // std::sqrt(3.0)/2.0;

}  // namespace yeet::control

#endif  // YEET_CONTROL_CONSTANTS_HPP