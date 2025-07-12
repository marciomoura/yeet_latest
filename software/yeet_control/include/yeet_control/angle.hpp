#ifndef YEET_CONTROL_ANGLE_HPP
#define YEET_CONTROL_ANGLE_HPP

#include <cassert>
#include <cmath>
#include <type_traits>

#include "constants.hpp"

namespace yeet::control {

/**
 * @brief Range type for angles between -π and π radians
 *
 * Used as a template parameter for the angle class to enforce
 * angle wrapping between -π and π radians.
 */
struct minus_pi_to_pi {
    static constexpr double max = pi<double>;
    static constexpr double min = -pi<double>;
};

/**
 * @brief Range type for angles between 0 and 2π radians
 *
 * Used as a template parameter for the angle class to enforce
 * angle wrapping between 0 and 2π radians.
 */
struct zero_to_two_pi {
    static constexpr double max = 2.0 * pi<double>;
    static constexpr double min = 0.0;
};

/**
 * @brief A class representing an angle with automatic range wrapping
 *
 * This class provides a type-safe way to work with angles, ensuring they stay within
 * a specified range. The range is determined by the template parameter RangeType,
 * which can be either minus_pi_to_pi or zero_to_two_pi.
 *
 * The class automatically wraps angles to stay within the specified range during
 * all operations, and provides common arithmetic operations and conversions.
 *
 * @tparam RangeType The range type that determines the valid angle range.
 *                   Must be either minus_pi_to_pi or zero_to_two_pi.
 */
template <typename RangeType = zero_to_two_pi>
class angle {
    static_assert(std::is_same_v<RangeType, minus_pi_to_pi> || std::is_same_v<RangeType, zero_to_two_pi>,
                  "RangeType must be either minus_pi_to_pi or zero_to_two_pi");

public:
    /**
     * @brief Constructs an angle with the specified value
     *
     * The input value will be automatically wrapped to the valid range
     * determined by RangeType.
     *
     * @param value The angle value in radians (default = 0.0)
     */
    constexpr angle<RangeType>(double value = 0.0) { set_value(value); }

    /**
     * @brief Sets the angle value with automatic range wrapping
     *
     * @param value The new angle value in radians
     */
    constexpr void set_value(double value) { _value = wrap_around(value); }

    /**
     * @brief Gets the angle value in degrees
     *
     * @return The angle in degrees, wrapped to the valid range
     */
    constexpr double get_degrees() const { return _value * 180.0 / pi<double>; }

    /**
     * @brief Gets the angle value in radians
     *
     * @return The angle in radians, wrapped to the valid range
     */
    constexpr double get_radians() const { return _value; }

    /**
     * @brief Gets the angle value in per unit
     *
     * @return double The angle in per unit, wrapped to the valid range
     */
    constexpr double get_pu() const {
        if constexpr (std::is_same_v<RangeType, zero_to_two_pi>) {
            return _value / two_pi<double>;
        } else if constexpr (std::is_same_v<RangeType, minus_pi_to_pi>) {
            return (_value + pi<double>) / two_pi<double>;
        }
    }

    /**
     * @brief Gets the angle value in radians
     *
     * This method allows the angle object to be used as a function object.
     *
     * @return The angle in radians, wrapped to the valid range
     */
    constexpr operator double() const { return _value; }

    /**
     * @brief Negation operator for the angle value
     *
     * @return A new angle representing the negated value
     */
    constexpr angle<RangeType> operator-() const { return angle(-_value); }

    /**
     * @brief Adds two angles
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to add
     * @return A new angle representing the sum
     */
    constexpr angle<RangeType> operator+(const angle<RangeType>& other) const { return angle(_value + other._value); }

    /**
     * @brief Subtracts two angles
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to subtract
     * @return A new angle representing the difference
     */
    constexpr angle<RangeType> operator-(const angle<RangeType>& other) const { return angle(_value - other._value); }

    /**
     * @brief Adds an angle to this one
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to add
     * @return Reference to this angle after addition
     */
    constexpr angle<RangeType>& operator+=(const angle<RangeType>& other) {
        set_value(_value + other._value);
        return *this;
    }

    /**
     * @brief Subtracts an angle from this one
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to subtract
     * @return Reference to this angle after subtraction
     */
    constexpr angle<RangeType>& operator-=(const angle<RangeType>& other) {
        set_value(_value - other._value);
        return *this;
    }

    /**
     * @brief Multiplies the angle by a scalar
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param scalar The scalar value to multiply by
     * @return A new angle representing the product
     */
    constexpr angle<RangeType> operator*(double scalar) const { return angle(_value * scalar); }

    /**
     * @brief Compares two angles for equality
     *
     * Angles are considered equal if they differ by less than epsilon
     * or by less than epsilon from 2π.
     *
     * @param other The angle to compare with
     * @return true if the angles are equal, false otherwise
     */
    constexpr bool operator==(const angle<RangeType>& other) const {
        constexpr double epsilon = 1e-10;
        double diff = std::abs(_value - other._value);
        return diff < epsilon || std::abs(diff - 2 * pi<double>) < epsilon;
    }

    /**
     * @brief Compares two angles for inequality
     *
     * @param other The angle to compare with
     * @return true if the angles are not equal, false otherwise
     */
    constexpr bool operator!=(const angle<RangeType>& other) const { return !(*this == other); }

public:
    // Implement an operator < for angle comparison
    /**
     * @brief Compares two angles for less than
     *
     * This comparison is based on the angle value in radians.
     *
     * @param other The angle to compare with
     * @return true if this angle is less than the other, false otherwise
     */
    bool operator<(const angle<RangeType>& other) const {
        double this_wrapped = wrap_around(_value);
        double other_wrapped = wrap_around(other._value);
        return this_wrapped < other_wrapped;
    }

    /**
     * @brief Compares two angles for less than or equal to
     *
     * This comparison is based on the angle value in radians.
     *
     * @param other The angle to compare with
     * @return true if this angle is less than or equal to the other, false otherwise
     */
    bool operator<=(const angle<RangeType>& other) const { return *this < other || *this == other; }

    /**
     * @brief Compares two angles for greater than
     *
     * This comparison is based on the angle value in radians.
     *
     * @param other The angle to compare with
     * @return true if this angle is greater than the other, false otherwise
     */
    bool operator>(const angle<RangeType>& other) const { return other < *this; }

    /**
     * @brief Compares two angles for greater than or equal to
     *
     * This comparison is based on the angle value in radians.
     *
     * @param other The angle to compare with
     * @return true if this angle is greater than or equal to the other, false otherwise
     */
    bool operator>=(const angle<RangeType>& other) const { return *this > other || *this == other; }

private:
    /**
     * @brief Normalizes an angle value to the proper range for comparison
     *
     * This function applies the same wrapping logic as set_value to ensure
     * consistent angle normalization across all comparison operations.
     *
     * @param value The angle value to normalize
     * @return The normalized angle value within the range specified by RangeType
     */
    static double wrap_around(double value) {
        // First reduce to [-2π, 2π] range
        while (value > two_pi<double>) {
            value -= two_pi<double>;
        }
        while (value < -two_pi<double>) {
            value += two_pi<double>;
        }

        // Apply range-specific wrapping
        if constexpr (std::is_same_v<RangeType, zero_to_two_pi>) {
            if (value < 0.0) {
                value += two_pi<double>;
            }
        } else if constexpr (std::is_same_v<RangeType, minus_pi_to_pi>) {
            if (value > pi<double>) {
                value -= two_pi<double>;
            } else if (value <= -pi<double>) {
                value += two_pi<double>;
            }
        }
        return value;
    }

    double _value{};  ///< The stored angle value in radians
};

// Declare extern template to prevent implicit instantiation
extern template class angle<minus_pi_to_pi>;
extern template class angle<zero_to_two_pi>;

}  // namespace yeet::control

#endif  // YEET_CONTROL_ANGLE_HPP