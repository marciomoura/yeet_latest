#ifndef YEET_CONTROL_LIMIT_HPP
#define YEET_CONTROL_LIMIT_HPP

#include <algorithm>
#include <cassert>
#include <type_traits>

namespace yeet::control {

// Result type for operations that need to report limiting status
template <typename T>
struct limit_result {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

    T value;           // The limited value
    bool was_limited;  // True if the value was modified
};

template <typename T>
class limit {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

public:
    // Limits a value to be at or below a maximum threshold
    static T upper(T value, T max_value) { return std::min(value, max_value); }

    // Limits a value to be at or above a minimum threshold
    static T lower(T value, T min_value) { return std::max(value, min_value); }

    // Limits a value to be within a range [min_value, max_value]
    static T range(T value, T min_value, T max_value) {
        assert(max_value > min_value && "Wrong range limits");
        return std::min(std::max(value, min_value), max_value);
    }

    // Range limiting with status reporting using epsilon comparison
    static limit_result<T> range_with_status(T value, T min_value, T max_value) {
        T original = value;
        T limited = range(value, min_value, max_value);
        if constexpr (std::is_floating_point_v<T>) {
            return {limited, std::abs(limited - original) > default_epsilon};
        } else {
            return {limited, limited != original};
        }
    }

private:
    static constexpr T default_epsilon = T(1e-10);
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_LIMIT_HPP