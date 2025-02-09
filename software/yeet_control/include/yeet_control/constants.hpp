#ifndef YEET_CONTROL_CONSTANTS_HPP
#define YEET_CONTROL_CONSTANTS_HPP

namespace yeet::control {

template <typename T>
static constexpr T pi = T(3.1415926535897932385);

template <typename T>
static constexpr T two_pi = T(2.0) * pi<T>;

constexpr double sqrt_3 = 1.73205080757;        // std::sqrt(3.0);
constexpr double sqrt_3_over_2 = sqrt_3 / 2.0;  // std::sqrt(3.0)/2.0;

}  // namespace yeet::control

#endif  // YEET_CONTROL_CONSTANTS_HPP