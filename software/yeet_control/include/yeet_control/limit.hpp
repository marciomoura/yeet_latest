#ifndef YEET_CONTROL_LIMIT_HPP
#define YEET_CONTROL_LIMIT_HPP

#include <algorithm>
#include <cassert>
#include <type_traits>

#include "types.hpp"
#include "units.hpp"

namespace yeet::control {

// Result type for operations that need to report limiting status
template <typename T>
struct limit_result {
    T value;           // The limited value
    bool was_limited;  // True if the value was modified
};

template <typename T>
class limit {
public:
    // Limits a value to be at or below a maximum threshold
    static T upper(T value, T max_value) { return std::min(value, max_value); }

    // Limits a value to be at or above a minimum threshold
    static T lower(T value, T min_value) { return std::max(value, min_value); }

    // Limits a value to be within a range [min_value, max_value]
    static T range(T value, T min_value, T max_value)
    {
        assert(max_value > min_value && "Wrong range limits");
        return std::min(std::max(value, min_value), max_value);
    }

    // Detects if a value exceeds the upper limit without performing any limiting
    static bool is_above_upper_limit(T value, T max_value)
    {
        if constexpr (yeet::use_epsilon_comparison_v<T>) {
            auto diff = value - max_value;
            if constexpr (yeet::is_quantity_v<T>) {
                return diff.value() > real_t(1e-10);
            }
            else {
                return diff > default_epsilon;
            }
        }
        else {
            return value > max_value;
        }
    }

    // Range limiting with status reporting using epsilon comparison
    static limit_result<T> range_with_status(T value, T min_value, T max_value)
    {
        T original = value;
        T limited = range(value, min_value, max_value);
        if constexpr (yeet::use_epsilon_comparison_v<T>) {
            if constexpr (yeet::is_quantity_v<T>) {
                auto diff = abs(limited - original);
                return {limited, diff.value() > real_t(1e-10)};
            }
            else {
                return {limited, std::abs(limited - original) > default_epsilon};
            }
        }
        else {
            return {limited, limited != original};
        }
    }

    // Range limiting with upper limit status reporting
    // Returns true in was_limited only if the upper limit was hit
    static limit_result<T> range_with_upper_limit_status(T value, T min_value, T max_value)
    {
        T limited = range(value, min_value, max_value);
        bool hit_upper_limit = false;

        if constexpr (yeet::use_epsilon_comparison_v<T>) {
            if constexpr (yeet::is_quantity_v<T>) {
                auto diff = value - max_value;
                hit_upper_limit = diff.value() > real_t(1e-10);
            }
            else {
                hit_upper_limit = (value - max_value) > default_epsilon;
            }
        }
        else {
            hit_upper_limit = value > max_value;
        }

        return {limited, hit_upper_limit};
    }

private:
    static constexpr T default_epsilon = T(1e-10);
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_LIMIT_HPP