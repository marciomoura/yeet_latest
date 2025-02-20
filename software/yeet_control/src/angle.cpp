#include "yeet_control/angle.hpp"

namespace yeet::control {

template <typename RangeType>
angle<RangeType>::angle(double value) {
    set_value(value);
}

template <typename RangeType>
void angle<RangeType>::set_value(double value) {
    if constexpr (std::is_same_v<RangeType, zero_to_two_pi>) {
        if (value < 0.0) {
            value += two_pi<double>;
        } else if (value >= two_pi<double>) {
            value -= two_pi<double>;
        }
    } else if constexpr (std::is_same_v<RangeType, minus_pi_to_pi>) {
        if (value >= pi<double>) {
            value -= two_pi<double>;
        } else if (value < -pi<double>) {
            value += two_pi<double>;
        }
    }

    assert(value >= RangeType::min && value <= RangeType::max);
    _value = value;
}

template <typename RangeType>
double angle<RangeType>::get_degrees() const {
    return _value * 180.0 / pi<double>;
}

template <typename RangeType>
double angle<RangeType>::get_radians() const {
    return _value;
}

template <typename RangeType>
angle<RangeType> angle<RangeType>::operator-() const {
    return angle(-_value);
}

template <typename RangeType>
angle<RangeType> angle<RangeType>::operator+(const angle& other) const {
    return angle(_value + other._value);
}

template <typename RangeType>
angle<RangeType> angle<RangeType>::operator-(const angle& other) const {
    return angle(_value - other._value);
}

template <typename RangeType>
angle<RangeType>& angle<RangeType>::operator+=(const angle& other) {
    set_value(_value + other._value);
    return *this;
}

template <typename RangeType>
angle<RangeType>& angle<RangeType>::operator-=(const angle& other) {
    set_value(_value - other._value);
    return *this;
}

template <typename RangeType>
angle<RangeType> angle<RangeType>::operator*(double scalar) const {
    return angle(_value * scalar);
}

template <typename RangeType>
bool angle<RangeType>::operator==(const angle& other) const {
    constexpr double epsilon = 1e-10;
    double diff = std::abs(_value - other._value);
    return diff < epsilon || std::abs(diff - 2 * pi<double>) < epsilon;
}

template <typename RangeType>
bool angle<RangeType>::operator!=(const angle& other) const {
    return !(*this == other);
}

// Explicit instantiation of the template for the known types
template class angle<minus_pi_to_pi>;
template class angle<zero_to_two_pi>;

}  // namespace yeet::control