#ifndef YEET_CONTROL_COORDINATES_HPP
#define YEET_CONTROL_COORDINATES_HPP

#include <array>
#include <cmath>

#include "angle.hpp"
#include "constants.hpp"

namespace yeet::control {

// Forward declarations
template <typename T>
class abc;
template <typename T>
class alphabeta;
template <typename T>
class dq;

/**
 * @brief Three-phase (abc) coordinate frame
 *
 * Represents quantities in the three-phase stationary frame.
 */
template <typename T>
class abc {
public:
    // Constructors
    constexpr abc() = default;
    constexpr abc(const std::array<T, 3>& values) : _values(values) {}
    constexpr abc(T a, T b, T c) : _values{a, b, c} {}
    constexpr abc(const abc<T>& other) : _values(other._values) {}
    constexpr abc(abc<T>&& other) : _values(std::move(other._values)) {}

    constexpr abc<T>& operator=(const abc<T>& other) {
        _values = other._values;
        return *this;
    }

    constexpr abc<T>& operator=(abc<T>&& other) {
        _values = std::move(other._values);
        return *this;
    }

    // Array access
    constexpr T& operator[](size_t idx) { return _values[idx]; }
    constexpr const T& operator[](size_t idx) const { return _values[idx]; }

    // Conversion to array
    constexpr operator std::array<T, 3>() const { return _values; }
    constexpr std::array<T, 3> array() const { return _values; }

    // Conversion to other frames
    constexpr alphabeta<T> to_alphabeta() const;
    constexpr dq<T> to_dq(const angle<zero_to_two_pi>& theta) const;

    // Rotation
    constexpr abc<T> rotate(const angle<zero_to_two_pi>& angle) const {
        // Rotate by converting to alphabeta, rotating there, and converting back
        return to_alphabeta().rotate(angle).to_abc();
    }

    // Addition operator
    constexpr abc<T> operator+(const abc<T>& other) const {
        return abc<T>{_values[0] + other._values[0], _values[1] + other._values[1], _values[2] + other._values[2]};
    }

    // Subtraction operator
    constexpr abc<T> operator-(const abc<T>& other) const {
        return abc<T>{_values[0] - other._values[0], _values[1] - other._values[1], _values[2] - other._values[2]};
    }

    // Compound addition operator
    constexpr abc<T>& operator+=(const abc<T>& other) {
        _values[0] += other._values[0];
        _values[1] += other._values[1];
        _values[2] += other._values[2];
        return *this;
    }

    // Compound subtraction operator
    constexpr abc<T>& operator-=(const abc<T>& other) {
        _values[0] -= other._values[0];
        _values[1] -= other._values[1];
        _values[2] -= other._values[2];
        return *this;
    }

    // Scalar multiplication
    constexpr abc<T> operator*(T scalar) const {
        return abc<T>{_values[0] * scalar, _values[1] * scalar, _values[2] * scalar};
    }

    // Scalar division
    constexpr abc<T> operator/(T scalar) const {
        return abc<T>{_values[0] / scalar, _values[1] / scalar, _values[2] / scalar};
    }

    // Compound scalar multiplication
    constexpr abc<T>& operator*=(T scalar) {
        _values[0] *= scalar;
        _values[1] *= scalar;
        _values[2] *= scalar;
        return *this;
    }

    // Compound scalar division
    constexpr abc<T>& operator/=(T scalar) {
        _values[0] /= scalar;
        _values[1] /= scalar;
        _values[2] /= scalar;
        return *this;
    }

    // Negation operator
    constexpr abc<T> operator-() const { return abc<T>{-_values[0], -_values[1], -_values[2]}; }

    auto begin() { return _values.begin(); }
    auto end() { return _values.end(); }
    auto begin() const { return _values.begin(); }
    auto end() const { return _values.end(); }
    std::size_t size() const { return _values.size(); }

private:
    std::array<T, 3> _values{};
};

// Non-member scalar multiplication (for scalar * abc)
template <typename T>
constexpr abc<T> operator*(T scalar, const abc<T>& coords) {
    return coords * scalar;
}

/**
 * @brief Two-phase stationary (αβ) coordinate frame
 *
 * Represents quantities in the two-phase stationary frame.
 */
template <typename T>
class alphabeta {
public:
    // Constructors
    constexpr alphabeta() = default;
    constexpr alphabeta(const std::array<T, 2>& values) : _values(values) {}
    constexpr alphabeta(T alpha, T beta) : _values{alpha, beta} {}

    constexpr alphabeta(const alphabeta<T>& other) : _values(other._values) {}
    constexpr alphabeta(alphabeta<T>&& other) : _values(std::move(other._values)) {}

    constexpr alphabeta<T>& operator=(const alphabeta<T>& other) {
        _values = other._values;
        return *this;
    }

    constexpr alphabeta<T>& operator=(alphabeta<T>&& other) {
        _values = std::move(other._values);
        return *this;
    }

    // Array access
    constexpr T& operator[](size_t idx) { return _values[idx]; }
    constexpr const T& operator[](size_t idx) const { return _values[idx]; }

    // Conversion to array
    constexpr operator std::array<T, 2>() const { return _values; }
    constexpr std::array<T, 2> array() const { return _values; }

    // Conversion to other frames
    constexpr abc<T> to_abc() const;
    constexpr dq<T> to_dq(const angle<zero_to_two_pi>& theta) const;

    // Magnitude calculation
    constexpr T magnitude() const { return std::sqrt(_values[0] * _values[0] + _values[1] * _values[1]); }

    // Phase calculation
    constexpr angle<zero_to_two_pi> phase() const { return angle<zero_to_two_pi>(std::atan2(_values[1], _values[0])); }

    // Counter-clockwise rotation:
    //    [cos θ  -sin θ] [α] = [α cos θ - β sin θ]
    //    [sin θ   cos θ] [β]   [α sin θ + β cos θ]
    constexpr alphabeta<T> rotate_counter_clockwise(const angle<zero_to_two_pi>& theta) const {
        const double cos_angle = std::cos(theta.get_radians());
        const double sin_angle = std::sin(theta.get_radians());
        return alphabeta<T>{_values[0] * cos_angle - _values[1] * sin_angle,
                            _values[0] * sin_angle + _values[1] * cos_angle};
    }

    constexpr alphabeta<T> rotate_clockwise(const angle<zero_to_two_pi>& theta) const {
        return rotate_counter_clockwise(angle<zero_to_two_pi>(-theta.get_radians()));
    }

    // Addition operator
    constexpr alphabeta<T> operator+(const alphabeta<T>& other) const {
        return alphabeta<T>{_values[0] + other._values[0], _values[1] + other._values[1]};
    }

    // Subtraction operator
    constexpr alphabeta<T> operator-(const alphabeta<T>& other) const {
        return alphabeta<T>{_values[0] - other._values[0], _values[1] - other._values[1]};
    }

    // Compound addition operator
    constexpr alphabeta<T>& operator+=(const alphabeta<T>& other) {
        _values[0] += other._values[0];
        _values[1] += other._values[1];
        return *this;
    }

    // Compound subtraction operator
    constexpr alphabeta<T>& operator-=(const alphabeta<T>& other) {
        _values[0] -= other._values[0];
        _values[1] -= other._values[1];
        return *this;
    }

    // Scalar multiplication
    constexpr alphabeta<T> operator*(T scalar) const { return alphabeta<T>{_values[0] * scalar, _values[1] * scalar}; }

    // Scalar division
    constexpr alphabeta<T> operator/(T scalar) const { return alphabeta<T>{_values[0] / scalar, _values[1] / scalar}; }

    // Compound scalar multiplication
    constexpr alphabeta<T>& operator*=(T scalar) {
        _values[0] *= scalar;
        _values[1] *= scalar;
        return *this;
    }

    // Compound scalar division
    constexpr alphabeta<T>& operator/=(T scalar) {
        _values[0] /= scalar;
        _values[1] /= scalar;
        return *this;
    }

    // Negation operator
    constexpr alphabeta<T> operator-() const { return alphabeta<T>{-_values[0], -_values[1]}; }

    auto begin() { return _values.begin(); }
    auto end() { return _values.end(); }
    auto begin() const { return _values.begin(); }
    auto end() const { return _values.end(); }
    std::size_t size() const { return _values.size(); }

private:
    std::array<T, 2> _values{};
};

// Non-member scalar multiplication (for scalar * alphabeta)
template <typename T>
constexpr alphabeta<T> operator*(T scalar, const alphabeta<T>& coords) {
    return coords * scalar;
}

/**
 * @brief Rotating (dq) coordinate frame
 *
 * Represents quantities in the rotating reference frame.
 */
template <typename T>
class dq {
public:
    // Constructors
    constexpr dq() = default;
    constexpr dq(const std::array<T, 2>& values) : _values(values) {}
    constexpr dq(T d, T q) : _values{d, q} {}

    constexpr dq(const dq<T>& other) : _values(other._values) {}
    constexpr dq(dq<T>&& other) : _values(std::move(other._values)) {}

    constexpr dq<T>& operator=(const dq<T>& other) {
        _values = other._values;
        return *this;
    }

    constexpr dq<T>& operator=(dq<T>&& other) {
        _values = std::move(other._values);
        return *this;
    }

    // Array access
    constexpr T& operator[](size_t idx) { return _values[idx]; }
    constexpr const T& operator[](size_t idx) const { return _values[idx]; }

    // Conversion to array
    constexpr operator std::array<T, 2>() const { return _values; }
    constexpr std::array<T, 2> array() const { return _values; }

    // Conversion to other frames
    constexpr alphabeta<T> to_alphabeta(const angle<zero_to_two_pi>& theta) const;
    constexpr abc<T> to_abc(const angle<zero_to_two_pi>& theta) const;

    // Magnitude calculation
    constexpr T magnitude() const { return std::sqrt(_values[0] * _values[0] + _values[1] * _values[1]); }

    constexpr angle<zero_to_two_pi> phase(const angle<zero_to_two_pi>& frame) const {
        return frame + angle<zero_to_two_pi>(std::atan2(_values[1], _values[0]));
    }

    // Counter-clockwise rotation:
    //    [cos θ  -sin θ] [α] = [α cos θ - β sin θ]
    //    [sin θ   cos θ] [β]   [α sin θ + β cos θ]
    constexpr dq<T> rotate_counter_clockwise(const angle<zero_to_two_pi>& theta) const {
        const double cos_angle = std::cos(theta.get_radians());
        const double sin_angle = std::sin(theta.get_radians());
        return dq<T>{_values[0] * cos_angle - _values[1] * sin_angle, _values[0] * sin_angle + _values[1] * cos_angle};
    }

    constexpr dq<T> rotate_clockwise(const angle<zero_to_two_pi>& theta) const {
        return rotate_counter_clockwise(angle<zero_to_two_pi>(-theta.get_radians()));
    }

    // Addition operator
    constexpr dq<T> operator+(const dq<T>& other) const {
        return dq<T>{_values[0] + other._values[0], _values[1] + other._values[1]};
    }

    // Subtraction operator
    constexpr dq<T> operator-(const dq<T>& other) const {
        return dq<T>{_values[0] - other._values[0], _values[1] - other._values[1]};
    }

    // Compound addition operator
    constexpr dq<T>& operator+=(const dq<T>& other) {
        _values[0] += other._values[0];
        _values[1] += other._values[1];
        return *this;
    }

    // Compound subtraction operator
    constexpr dq<T>& operator-=(const dq<T>& other) {
        _values[0] -= other._values[0];
        _values[1] -= other._values[1];
        return *this;
    }

    // Scalar multiplication
    constexpr dq<T> operator*(T scalar) const { return dq<T>{_values[0] * scalar, _values[1] * scalar}; }

    // Scalar division
    constexpr dq<T> operator/(T scalar) const { return dq<T>{_values[0] / scalar, _values[1] / scalar}; }

    // Compound scalar multiplication
    constexpr dq<T>& operator*=(T scalar) {
        _values[0] *= scalar;
        _values[1] *= scalar;
        return *this;
    }

    // Compound scalar division
    constexpr dq<T>& operator/=(T scalar) {
        _values[0] /= scalar;
        _values[1] /= scalar;
        return *this;
    }

    // Negation operator
    constexpr dq<T> operator-() const { return dq<T>{-_values[0], -_values[1]}; }

    auto begin() { return _values.begin(); }
    auto end() { return _values.end(); }
    auto begin() const { return _values.begin(); }
    auto end() const { return _values.end(); }
    std::size_t size() const { return _values.size(); }

private:
    std::array<T, 2> _values{};
};

// Non-member scalar multiplication (for scalar * dq)
template <typename T>
constexpr dq<T> operator*(T scalar, const dq<T>& coords) {
    return coords * scalar;
}

// Implementation of conversion functions

template <typename T>
constexpr alphabeta<T> abc<T>::to_alphabeta() const {
    return alphabeta<T>{2.0 / 3.0 * (_values[0] - 0.5 * _values[1] - 0.5 * _values[2]),
                        sqrt_3 / 3.0 * (_values[1] - _values[2])};
}

template <typename T>
constexpr dq<T> abc<T>::to_dq(const angle<zero_to_two_pi>& theta) const {
    return to_alphabeta().to_dq(theta);
}

template <typename T>
constexpr abc<T> alphabeta<T>::to_abc() const {
    return abc<T>{_values[0], -0.5 * _values[0] + sqrt_3 / 2.0 * _values[1],
                  -0.5 * _values[0] - sqrt_3 / 2.0 * _values[1]};
}

template <typename T>
constexpr dq<T> alphabeta<T>::to_dq(const angle<zero_to_two_pi>& theta) const {
    const double cos_theta = std::cos(theta.get_radians());
    const double sin_theta = std::sin(theta.get_radians());
    return dq<T>{_values[0] * cos_theta + _values[1] * sin_theta, -_values[0] * sin_theta + _values[1] * cos_theta};
}

template <typename T>
constexpr alphabeta<T> dq<T>::to_alphabeta(const angle<zero_to_two_pi>& theta) const {
    const double cos_theta = std::cos(theta.get_radians());
    const double sin_theta = std::sin(theta.get_radians());
    return alphabeta<T>{_values[0] * cos_theta - _values[1] * sin_theta,
                        _values[0] * sin_theta + _values[1] * cos_theta};
}

template <typename T>
constexpr abc<T> dq<T>::to_abc(const angle<zero_to_two_pi>& theta) const {
    return to_alphabeta(theta).to_abc();
}

// Utility functions for creating coordinates

template <typename T>
constexpr abc<T> make_abc(T magnitude, const angle<zero_to_two_pi>& phase) {
    constexpr double TWO_PI_THIRDS = 2.0 * pi<double> / 3.0;
    const double angle_rad = phase.get_radians();
    return abc<T>{magnitude * std::cos(angle_rad), magnitude * std::cos(angle_rad - TWO_PI_THIRDS),
                  magnitude * std::cos(angle_rad + TWO_PI_THIRDS)};
}

template <typename T>
constexpr alphabeta<T> make_alphabeta(T magnitude, const angle<zero_to_two_pi>& phase) {
    const double angle_rad = phase.get_radians();
    return alphabeta<T>{magnitude * std::cos(angle_rad), magnitude * std::sin(angle_rad)};
}

template <typename T>
constexpr dq<T> make_dq(T d, T q) {
    return dq<T>{d, q};
}

}  // namespace yeet::control

#endif  // YEET_CONTROL_COORDINATES_HPP