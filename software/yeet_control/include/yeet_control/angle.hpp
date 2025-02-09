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
    explicit angle(double value = 0.0);

    /**
     * @brief Sets the angle value with automatic range wrapping
     *
     * @param value The new angle value in radians
     */
    void set_value(double value);

    /**
     * @brief Gets the angle value in degrees
     *
     * @return The angle in degrees, wrapped to the valid range
     */
    double get_degrees() const;

    /**
     * @brief Gets the angle value in radians
     *
     * @return The angle in radians, wrapped to the valid range
     */
    double get_radians() const;

    /**
     * @brief Gets the angle value in radians
     *
     * This method allows the angle object to be used as a function object.
     *
     * @return The angle in radians, wrapped to the valid range
     */
    operator double() const { return _value; }

    /**
     * @brief Adds two angles
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to add
     * @return A new angle representing the sum
     */
    angle operator+(const angle& other) const;

    /**
     * @brief Subtracts two angles
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to subtract
     * @return A new angle representing the difference
     */
    angle operator-(const angle& other) const;

    /**
     * @brief Adds an angle to this one
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to add
     * @return Reference to this angle after addition
     */
    angle& operator+=(const angle& other);

    /**
     * @brief Subtracts an angle from this one
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param other The angle to subtract
     * @return Reference to this angle after subtraction
     */
    angle& operator-=(const angle& other);

    /**
     * @brief Multiplies the angle by a scalar
     *
     * The result is automatically wrapped to the valid range.
     *
     * @param scalar The scalar value to multiply by
     * @return A new angle representing the product
     */
    angle operator*(double scalar) const;

    /**
     * @brief Compares two angles for equality
     *
     * Angles are considered equal if they differ by less than epsilon
     * or by less than epsilon from 2π.
     *
     * @param other The angle to compare with
     * @return true if the angles are equal, false otherwise
     */
    bool operator==(const angle& other) const;

    /**
     * @brief Compares two angles for inequality
     *
     * @param other The angle to compare with
     * @return true if the angles are not equal, false otherwise
     */
    bool operator!=(const angle& other) const;

private:
    double _value{};  ///< The stored angle value in radians
};

// Declare extern template to prevent implicit instantiation
extern template class angle<minus_pi_to_pi>;
extern template class angle<zero_to_two_pi>;

}  // namespace yeet::control

#endif  // YEET_CONTROL_ANGLE_HPP