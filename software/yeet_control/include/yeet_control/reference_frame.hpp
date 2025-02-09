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
    abc() : _values{0, 0, 0} {}
    explicit abc(const std::array<T, 3>& values) : _values(values) {}
    abc(T a, T b, T c) : _values{a, b, c} {}

    // Array access
    T& operator[](size_t idx) { return _values[idx]; }
    const T& operator[](size_t idx) const { return _values[idx]; }

    // Conversion to array
    operator std::array<T, 3>() const { return _values; }
    std::array<T, 3> array() const { return _values; }

    // Conversion to other frames
    alphabeta<T> to_alphabeta() const;
    dq<T> to_dq(const angle<zero_to_two_pi>& theta) const;

    // Rotation
    abc<T> rotate(const angle<zero_to_two_pi>& angle) const {
        // Rotate by converting to alphabeta, rotating there, and converting back
        return to_alphabeta().rotate(angle).to_abc();
    }

    // Addition operator
    abc<T> operator+(const abc<T>& other) const {
        return abc<T>{_values[0] + other._values[0], _values[1] + other._values[1], _values[2] + other._values[2]};
    }

    // Subtraction operator
    abc<T> operator-(const abc<T>& other) const {
        return abc<T>{_values[0] - other._values[0], _values[1] - other._values[1], _values[2] - other._values[2]};
    }

    // Compound addition operator
    abc<T>& operator+=(const abc<T>& other) {
        _values[0] += other._values[0];
        _values[1] += other._values[1];
        _values[2] += other._values[2];
        return *this;
    }

    // Compound subtraction operator
    abc<T>& operator-=(const abc<T>& other) {
        _values[0] -= other._values[0];
        _values[1] -= other._values[1];
        _values[2] -= other._values[2];
        return *this;
    }

    // Scalar multiplication
    abc<T> operator*(T scalar) const { return abc<T>{_values[0] * scalar, _values[1] * scalar, _values[2] * scalar}; }

    // Scalar division
    abc<T> operator/(T scalar) const { return abc<T>{_values[0] / scalar, _values[1] / scalar, _values[2] / scalar}; }

    // Compound scalar multiplication
    abc<T>& operator*=(T scalar) {
        _values[0] *= scalar;
        _values[1] *= scalar;
        _values[2] *= scalar;
        return *this;
    }

    // Compound scalar division
    abc<T>& operator/=(T scalar) {
        _values[0] /= scalar;
        _values[1] /= scalar;
        _values[2] /= scalar;
        return *this;
    }

private:
    std::array<T, 3> _values;
};

// Non-member scalar multiplication (for scalar * abc)
template <typename T>
abc<T> operator*(T scalar, const abc<T>& coords) {
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
    alphabeta() = default;
    explicit alphabeta(const std::array<T, 2>& values) : _values(values) {}
    alphabeta(T alpha, T beta) : _values{alpha, beta} {}

    // Array access
    T& operator[](size_t idx) { return _values[idx]; }
    const T& operator[](size_t idx) const { return _values[idx]; }

    // Conversion to array
    operator std::array<T, 2>() const { return _values; }
    std::array<T, 2> array() const { return _values; }

    // Conversion to other frames
    abc<T> to_abc() const;
    dq<T> to_dq(const angle<zero_to_two_pi>& theta) const;

    // Magnitude calculation
    T magnitude() const { return std::sqrt(_values[0] * _values[0] + _values[1] * _values[1]); }

    // Phase calculation
    angle<zero_to_two_pi> phase() const { return angle<zero_to_two_pi>(std::atan2(_values[1], _values[0])); }

    // Counter-clockwise rotation:
    //    [cos θ  -sin θ] [α] = [α cos θ - β sin θ]
    //    [sin θ   cos θ] [β]   [α sin θ + β cos θ]
    alphabeta<T> rotate_counter_clockwise(const angle<zero_to_two_pi>& theta) const {
        const double cos_angle = std::cos(theta.get_radians());
        const double sin_angle = std::sin(theta.get_radians());

        return alphabeta<T>{_values[0] * cos_angle - _values[1] * sin_angle,
                            _values[0] * sin_angle + _values[1] * cos_angle};
    }

    alphabeta<T> rotate_clockwise(const angle<zero_to_two_pi>& theta) const {
        return rotate_counter_clockwise(angle<zero_to_two_pi>(-theta.get_radians()));
    }

    // Addition operator
    alphabeta<T> operator+(const alphabeta<T>& other) const {
        return alphabeta<T>{_values[0] + other._values[0], _values[1] + other._values[1]};
    }

    // Subtraction operator
    alphabeta<T> operator-(const alphabeta<T>& other) const {
        return alphabeta<T>{_values[0] - other._values[0], _values[1] - other._values[1]};
    }

    // Compound addition operator
    alphabeta<T>& operator+=(const alphabeta<T>& other) {
        _values[0] += other._values[0];
        _values[1] += other._values[1];
        return *this;
    }

    // Compound subtraction operator
    alphabeta<T>& operator-=(const alphabeta<T>& other) {
        _values[0] -= other._values[0];
        _values[1] -= other._values[1];
        return *this;
    }

    // Scalar multiplication
    alphabeta<T> operator*(T scalar) const { return alphabeta<T>{_values[0] * scalar, _values[1] * scalar}; }

    // Scalar division
    alphabeta<T> operator/(T scalar) const { return alphabeta<T>{_values[0] / scalar, _values[1] / scalar}; }

    // Compound scalar multiplication
    alphabeta<T>& operator*=(T scalar) {
        _values[0] *= scalar;
        _values[1] *= scalar;
        return *this;
    }

    // Compound scalar division
    alphabeta<T>& operator/=(T scalar) {
        _values[0] /= scalar;
        _values[1] /= scalar;
        return *this;
    }

private:
    std::array<T, 2> _values{};
};

// Non-member scalar multiplication (for scalar * alphabeta)
template <typename T>
alphabeta<T> operator*(T scalar, const alphabeta<T>& coords) {
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
    dq() {}
    explicit dq(const std::array<T, 2>& values) : _values(values) {}
    dq(T d, T q) : _values{d, q} {}

    // Array access
    T& operator[](size_t idx) { return _values[idx]; }
    const T& operator[](size_t idx) const { return _values[idx]; }

    // Conversion to array
    operator std::array<T, 2>() const { return _values; }
    std::array<T, 2> array() const { return _values; }

    // Conversion to other frames
    alphabeta<T> to_alphabeta(const angle<zero_to_two_pi>& theta) const;
    abc<T> to_abc(const angle<zero_to_two_pi>& theta) const;

    // Magnitude calculation
    T magnitude() const { return std::sqrt(_values[0] * _values[0] + _values[1] * _values[1]); }

    angle<zero_to_two_pi> phase(angle<zero_to_two_pi> const& frame) const {
        return frame + angle<zero_to_two_pi>(std::atan2(_values[1], _values[0]));
    }

    // Counter-clockwise rotation:
    //    [cos θ  -sin θ] [α] = [α cos θ - β sin θ]
    //    [sin θ   cos θ] [β]   [α sin θ + β cos θ]
    dq<T> rotate_counter_clockwise(const angle<zero_to_two_pi>& theta) const {
        const double cos_angle = std::cos(theta.get_radians());
        const double sin_angle = std::sin(theta.get_radians());

        return dq<T>{_values[0] * cos_angle - _values[1] * sin_angle, _values[0] * sin_angle + _values[1] * cos_angle};
    }

    dq<T> rotate_clockwise(const angle<zero_to_two_pi>& theta) const {
        return rotate_counter_clockwise(angle<zero_to_two_pi>(-theta.get_radians()));
    }

    // Addition operator
    dq<T> operator+(const dq<T>& other) const {
        return dq<T>{_values[0] + other._values[0], _values[1] + other._values[1]};
    }

    // Subtraction operator
    dq<T> operator-(const dq<T>& other) const {
        return dq<T>{_values[0] - other._values[0], _values[1] - other._values[1]};
    }

    // Compound addition operator
    dq<T>& operator+=(const dq<T>& other) {
        _values[0] += other._values[0];
        _values[1] += other._values[1];
        return *this;
    }

    // Compound subtraction operator
    dq<T>& operator-=(const dq<T>& other) {
        _values[0] -= other._values[0];
        _values[1] -= other._values[1];
        return *this;
    }

    // Scalar multiplication
    dq<T> operator*(T scalar) const { return dq<T>{_values[0] * scalar, _values[1] * scalar}; }

    // Scalar division
    dq<T> operator/(T scalar) const { return dq<T>{_values[0] / scalar, _values[1] / scalar}; }

    // Compound scalar multiplication
    dq<T>& operator*=(T scalar) {
        _values[0] *= scalar;
        _values[1] *= scalar;
        return *this;
    }

    // Compound scalar division
    dq<T>& operator/=(T scalar) {
        _values[0] /= scalar;
        _values[1] /= scalar;
        return *this;
    }

private:
    std::array<T, 2> _values{};
};

// Non-member scalar multiplication (for scalar * dq)
template <typename T>
dq<T> operator*(T scalar, const dq<T>& coords) {
    return coords * scalar;
}

// Implementation of conversion functions

template <typename T>
alphabeta<T> abc<T>::to_alphabeta() const {
    return alphabeta<T>{2.0 / 3.0 * (_values[0] - 0.5 * _values[1] - 0.5 * _values[2]),
                        sqrt_3 / 3.0 * (_values[1] - _values[2])};
}

template <typename T>
dq<T> abc<T>::to_dq(const angle<zero_to_two_pi>& theta) const {
    return to_alphabeta().to_dq(theta);
}

template <typename T>
abc<T> alphabeta<T>::to_abc() const {
    return abc<T>{_values[0], -0.5 * _values[0] + sqrt_3 / 2.0 * _values[1],
                  -0.5 * _values[0] - sqrt_3 / 2.0 * _values[1]};
}

template <typename T>
dq<T> alphabeta<T>::to_dq(const angle<zero_to_two_pi>& theta) const {
    const double cos_theta = std::cos(theta.get_radians());
    const double sin_theta = std::sin(theta.get_radians());

    return dq<T>{_values[0] * cos_theta + _values[1] * sin_theta, -_values[0] * sin_theta + _values[1] * cos_theta};
}

template <typename T>
alphabeta<T> dq<T>::to_alphabeta(const angle<zero_to_two_pi>& theta) const {
    const double cos_theta = std::cos(theta.get_radians());
    const double sin_theta = std::sin(theta.get_radians());

    return alphabeta<T>{_values[0] * cos_theta - _values[1] * sin_theta,
                        _values[0] * sin_theta + _values[1] * cos_theta};
}

template <typename T>
abc<T> dq<T>::to_abc(const angle<zero_to_two_pi>& theta) const {
    return to_alphabeta(theta).to_abc();
}

// Utility functions for creating coordinates

template <typename T>
abc<T> make_abc(T magnitude, const angle<zero_to_two_pi>& phase) {
    constexpr double TWO_PI_THIRDS = 2.0 * pi<double> / 3.0;
    const double angle = phase.get_radians();

    return abc<T>{magnitude * std::cos(angle), magnitude * std::cos(angle - TWO_PI_THIRDS),
                  magnitude * std::cos(angle + TWO_PI_THIRDS)};
}

template <typename T>
alphabeta<T> make_alphabeta(T magnitude, const angle<zero_to_two_pi>& phase) {
    const double angle = phase.get_radians();
    return alphabeta<T>{magnitude * std::cos(angle), magnitude * std::sin(angle)};
}

template <typename T>
dq<T> make_dq(T d, T q) {
    return dq<T>{d, q};
}

}  // namespace yeet::control

#endif  // YEET_CONTROL_COORDINATES_HPP