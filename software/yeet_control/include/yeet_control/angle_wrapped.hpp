#pragma once

#include <cassert>
#include <cmath>

#include "constants.hpp"
#include "units.hpp"

namespace yeet {

/**
 * @brief A class representing an angle with automatic range wrapping to [0, 2π]
 *
 * This class provides a type-safe way to work with angles, ensuring they stay within
 * the [0, 2π] radians range. The class automatically wraps angles to stay within
 * this range during all operations, and provides common arithmetic operations and conversions.
 *
 * All angles are internally stored and wrapped to the range [0, 2π] radians.
 * This is particularly useful for applications where angles represent absolute positions
 * or rotations that should always be positive.
 */
class angle_wrapped {
public:
    /**
     * @brief Creates an angle from degrees
     *
     * @param degrees The angle value in degrees
     * @return A new angle object with the specified degree value
     */
    static constexpr angle_wrapped from_degrees(real_t degrees) { return angle_wrapped(degrees * pi / 180.0); }

    /**
     * @brief Creates an angle from radians
     *
     * @param radians The angle value in radians
     * @return A new angle object with the specified radian value
     */
    static constexpr angle_wrapped from_radians(angle_t radians) { return angle_wrapped(radians); }

    constexpr angle_wrapped() = default;
    constexpr angle_wrapped(angle_wrapped const&) = default;
    constexpr angle_wrapped(angle_wrapped&&) = default;
    constexpr angle_wrapped& operator=(angle_wrapped const&) = default;
    constexpr angle_wrapped& operator=(angle_wrapped&&) = default;
    ~angle_wrapped() = default;

    /**
     * @brief Constructs an angle with the specified value
     *
     * The input value will be automatically wrapped to the [0, 2π] range.
     *
     * @param value The angle value in radians
     */
    constexpr angle_wrapped(real_t value) : _value(wrap_around(angle_t{value})) {}

    /**
     * @brief Constructs an angle with the specified value
     *
     * The input value will be automatically wrapped to the [0, 2π] range.
     *
     * @param value The angle value in radians
     */
    constexpr angle_wrapped(angle_t value) : _value(wrap_around(value)) {}

    /**
     * @brief Sets the angle value with automatic range wrapping
     *
     * @param value The new angle value in radians
     */
    constexpr void set_value(angle_t value) { _value = wrap_around(value); }

    /**
     * @brief Gets the angle value in degrees
     *
     * @return The angle in degrees, wrapped to the [0, 360] range
     */
    constexpr real_t get_degrees() const noexcept { return rad_to_deg(_value); }

    /**
     * @brief Gets the angle value in radians
     *
     * @return The angle in radians, wrapped to the [0, 2π] range
     */
    constexpr angle_t get_radians() const noexcept { return _value; }

    /**
     * @brief Gets the angle value in per unit
     *
     * @return The angle in per unit, normalized to [0, 1] range where 0 corresponds to 0 and 1 to 2π
     */
    constexpr angle_pu_t get_pu() const noexcept { return to_pu(_value, angle_t{two_pi}); }

    /**
     * @brief Gets the angle value in radians
     *
     * This method allows the angle object to be used as a function object.
     *
     * @return The angle in radians, wrapped to the [0, 2π] range
     */
    constexpr operator angle_t() const noexcept { return _value; }

    /**
     * @brief Gets the angle value in radians
     *
     * This method allows the angle object to be used as a function object.
     *
     * @return The angle in radians, wrapped to the [0, 2π] range
     */
    constexpr angle_t operator()() const noexcept { return _value; }

    /**
     * @brief Negation operator for the angle value
     *
     * @return A new angle representing the negated value, wrapped to [0, 2π]
     */
    constexpr angle_wrapped operator-() const { return angle_wrapped(-_value); }

    // ==================== Arithmetic Operations with angle_wrapped ====================

    /**
     * @brief Adds two angles
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param other The angle to add
     * @return A new angle representing the sum
     */
    constexpr angle_wrapped operator+(const angle_wrapped& other) const { return angle_wrapped(_value + other._value); }

    /**
     * @brief Subtracts two angles
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param other The angle to subtract
     * @return A new angle representing the difference
     */
    constexpr angle_wrapped operator-(const angle_wrapped& other) const { return angle_wrapped(_value - other._value); }

    /**
     * @brief Adds an angle to this one
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param other The angle to add
     * @return Reference to this angle after addition
     */
    constexpr angle_wrapped& operator+=(const angle_wrapped& other)
    {
        set_value(_value + other._value);
        return *this;
    }

    /**
     * @brief Subtracts an angle from this one
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param other The angle to subtract
     * @return Reference to this angle after subtraction
     */
    constexpr angle_wrapped& operator-=(const angle_wrapped& other)
    {
        set_value(_value - other._value);
        return *this;
    }

    // ==================== Arithmetic Operations with angle_t ====================

    /**
     * @brief Adds an angle_t value to this angle
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param angle_value The angle_t value to add
     * @return A new angle representing the sum
     */
    constexpr angle_wrapped operator+(const angle_t& angle_value) const { return angle_wrapped(_value + angle_value); }

    /**
     * @brief Subtracts an angle_t value from this angle
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param angle_value The angle_t value to subtract
     * @return A new angle representing the difference
     */
    constexpr angle_wrapped operator-(const angle_t& angle_value) const { return angle_wrapped(_value - angle_value); }

    /**
     * @brief Adds an angle_t value to this angle
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param angle_value The angle_t value to add
     * @return Reference to this angle after addition
     */
    constexpr angle_wrapped& operator+=(const angle_t& angle_value)
    {
        set_value(_value + angle_value);
        return *this;
    }

    /**
     * @brief Subtracts an angle_t value from this angle
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param angle_value The angle_t value to subtract
     * @return Reference to this angle after subtraction
     */
    constexpr angle_wrapped& operator-=(const angle_t& angle_value)
    {
        set_value(_value - angle_value);
        return *this;
    }

    // ==================== Scalar Operations with angle_t ====================

    /**
     * @brief Multiplies the angle by an angle_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The angle_t scalar value to multiply by
     * @return A new angle representing the product
     */
    constexpr angle_wrapped operator*(const angle_t& scalar) const { return angle_wrapped(_value * scalar); }

    /**
     * @brief Divides the angle by an angle_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The angle_t scalar value to divide by
     * @return A new angle representing the quotient
     */
    constexpr angle_wrapped operator/(const angle_t& scalar) const { return angle_wrapped(_value / scalar); }

    /**
     * @brief Multiplies this angle by an angle_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The angle_t scalar value to multiply by
     * @return Reference to this angle after multiplication
     */
    constexpr angle_wrapped& operator*=(const angle_t& scalar)
    {
        set_value(_value * scalar);
        return *this;
    }

    /**
     * @brief Divides this angle by an angle_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The angle_t scalar value to divide by
     * @return Reference to this angle after division
     */
    constexpr angle_wrapped& operator/=(const angle_t& scalar)
    {
        set_value(_value / scalar);
        return *this;
    }

    // ==================== Scalar Operations with real_t ====================

    /**
     * @brief Multiplies the angle by a real_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The real_t scalar value to multiply by
     * @return A new angle representing the product
     */
    constexpr angle_wrapped operator*(const real_t& scalar) const { return angle_wrapped(_value * scalar); }

    /**
     * @brief Divides the angle by a real_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The real_t scalar value to divide by
     * @return A new angle representing the quotient
     */
    constexpr angle_wrapped operator/(const real_t& scalar) const { return angle_wrapped(_value / scalar); }

    /**
     * @brief Multiplies this angle by a real_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The real_t scalar value to multiply by
     * @return Reference to this angle after multiplication
     */
    constexpr angle_wrapped& operator*=(const real_t& scalar)
    {
        set_value(_value * scalar);
        return *this;
    }

    /**
     * @brief Divides this angle by a real_t scalar
     *
     * The result is automatically wrapped to the [0, 2π] range.
     *
     * @param scalar The real_t scalar value to divide by
     * @return Reference to this angle after division
     */
    constexpr angle_wrapped& operator/=(const real_t& scalar)
    {
        set_value(_value / scalar);
        return *this;
    }

    // ==================== Comparison Operations ====================

    /**
     * @brief Compares two angles for equality
     *
     * Angles are considered equal if they differ by less than epsilon.
     *
     * @param other The angle to compare with
     * @return true if the angles are equal, false otherwise
     */
    constexpr bool operator==(const angle_wrapped& other) const noexcept
    {
        constexpr angle_t epsilon = angle_t{1e-10};
        const angle_t diff = abs(_value - other._value);
        return diff < epsilon;
    }

    /**
     * @brief Compares two angles for inequality
     *
     * @param other The angle to compare with
     * @return true if the angles are not equal, false otherwise
     */
    constexpr bool operator!=(const angle_wrapped& other) const noexcept { return !(*this == other); }

    /**
     * @brief Compares two angles for less than
     *
     * This comparison is based on the wrapped angle value in radians.
     *
     * @param other The angle to compare with
     * @return true if this angle is less than the other, false otherwise
     */
    constexpr bool operator<(const angle_wrapped& other) const noexcept { return _value < other._value; }

    /**
     * @brief Compares two angles for less than or equal to
     *
     * @param other The angle to compare with
     * @return true if this angle is less than or equal to the other, false otherwise
     */
    constexpr bool operator<=(const angle_wrapped& other) const noexcept { return *this < other || *this == other; }

    /**
     * @brief Compares two angles for greater than
     *
     * @param other The angle to compare with
     * @return true if this angle is greater than the other, false otherwise
     */
    constexpr bool operator>(const angle_wrapped& other) const noexcept { return other < *this; }

    /**
     * @brief Compares two angles for greater than or equal to
     *
     * @param other The angle to compare with
     * @return true if this angle is greater than or equal to the other, false otherwise
     */
    constexpr bool operator>=(const angle_wrapped& other) const noexcept { return *this > other || *this == other; }

private:
    /**
     * @brief Normalizes an angle value to the [0, 2π] range
     *
     * This function wraps the input angle to ensure it falls within the valid range.
     * The wrapping uses the mathematical principle: angle_wrapped = angle - 2π * floor(angle / 2π)
     *
     * @param value The angle value to normalize
     * @return The normalized angle value within [0, 2π] range
     */
    static constexpr angle_t wrap_around(angle_t value)
    {
        // Fast path for values already in range
        if (value.value() >= 0.0 && value.value() < two_pi) {
            return value;
        }

        // Use constexpr-compatible mathematical wrapping
        // The formula works for both positive and negative values
        return value - angle_t{two_pi * std::floor(value / two_pi)};
    }

    angle_t _value{};  ///< The stored angle value in radians, always in [0, 2π] range
};

// ==================== Free Function Operators for Commutative Operations ====================

/**
 * @brief Multiplies a real_t scalar by an angle_wrapped
 *
 * This enables expressions like: real_t{2.0} * angle_wrapped{pi}
 *
 * @param scalar The real_t scalar value
 * @param angle The angle to multiply
 * @return A new angle representing the product
 */
constexpr angle_wrapped operator*(const real_t& scalar, const angle_wrapped& angle) { return angle * scalar; }

/**
 * @brief Multiplies an angle_t scalar by an angle_wrapped
 *
 * This enables expressions like: angle_t{2.0} * angle_wrapped{pi}
 *
 * @param scalar The angle_t scalar value
 * @param angle The angle to multiply
 * @return A new angle representing the product
 */
constexpr angle_wrapped operator*(const angle_t& scalar, const angle_wrapped& angle) { return angle * scalar; }

/**
 * @brief Adds an angle_t value to an angle_wrapped
 *
 * This enables expressions like: angle_t{pi} + angle_wrapped{pi/2}
 *
 * @param angle_value The angle_t value
 * @param angle The angle_wrapped to add to
 * @return A new angle representing the sum
 */
constexpr angle_wrapped operator+(const angle_t& angle_value, const angle_wrapped& angle)
{
    return angle + angle_value;
}

}  // namespace yeet
