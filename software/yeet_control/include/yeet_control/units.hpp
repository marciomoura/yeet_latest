#pragma once

#include <iostream>
#include <string>
#include <type_traits>

namespace yeet::control {

//==============================================================================
// R1.1: Dimensional Representation & R2.3: System Differentiation
//==============================================================================

/**
 * @brief Represents the 7 base SI dimensions via integer exponents.
 *
 * L: Length, M: Mass, T: Time, I: Current, Th: Temperature, N: Amount, J: Luminous Intensity
 */
template <int L, int M, int T, int I, int Th, int N, int J>
struct dimension {};

/**
 * @brief Tag types for differentiating between unit systems (R2.1, R2.2, R2.3)
 */
struct si {};
struct per_unit {};

//==============================================================================
// R1.2: Quantity Type & R1.3: Underlying Value Type
//==============================================================================

/**
 * @brief Represents a physical quantity with dimension, unit system, and value type.
 */
template <typename Dimension, typename UnitSystem, typename Representation>
class quantity {
public:
    using dimension = Dimension;
    using system = UnitSystem;
    using representation = Representation;

    /**
     * @brief Default constructor
     */
    constexpr quantity() : _value(0) {}

    /**
     * @brief Explicit constructor from a raw value
     */
    constexpr explicit quantity(Representation val) : _value(val) {}

    /**
     * @brief Accessor for the underlying value
     */
    constexpr Representation value() const { return _value; }

    /**
     * @brief Implicit conversion to the underlying value type
     */
    operator Representation() const { return _value; }

private:
    Representation _value;
};

//==============================================================================
// R3: Arithmetic Operations (Helper Metafunctions)
//==============================================================================

/**
 * @brief Metafunction to add exponents for multiplication
 */
template <typename D1, typename D2>
struct multiply_dimensions;

template <int L1, int M1, int T1, int I1, int Th1, int N1, int J1, int L2, int M2, int T2, int I2, int Th2, int N2,
          int J2>
struct multiply_dimensions<dimension<L1, M1, T1, I1, Th1, N1, J1>, dimension<L2, M2, T2, I2, Th2, N2, J2>> {
    using type = dimension<L1 + L2, M1 + M2, T1 + T2, I1 + I2, Th1 + Th2, N1 + N2, J1 + J2>;
};

/**
 * @brief Metafunction to subtract exponents for division
 */
template <typename D1, typename D2>
struct divide_dimensions;

template <int L1, int M1, int T1, int I1, int Th1, int N1, int J1, int L2, int M2, int T2, int I2, int Th2, int N2,
          int J2>
struct divide_dimensions<dimension<L1, M1, T1, I1, Th1, N1, J1>, dimension<L2, M2, T2, I2, Th2, N2, J2>> {
    using type = dimension<L1 - L2, M1 - M2, T1 - T2, I1 - I2, Th1 - Th2, N1 - N2, J1 - J2>;
};

//==============================================================================
// R3.1: Addition & Subtraction
// Enforces R1.4 (Dimension Mismatch) and R1.5 (System Mismatch) at compile time.
//==============================================================================

template <typename D, typename S, typename R>
constexpr quantity<D, S, R> operator+(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return quantity<D, S, R>(lhs.value() + rhs.value());
}

template <typename D, typename S, typename R>
constexpr quantity<D, S, R> operator-(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return quantity<D, S, R>(lhs.value() - rhs.value());
}

//==============================================================================
// R3.2: Multiplication & R3.3: Division
//==============================================================================

template <typename D1, typename D2, typename S, typename R1, typename R2>
constexpr auto operator*(const quantity<D1, S, R1>& lhs, const quantity<D2, S, R2>& rhs) {
    using result_dim = typename multiply_dimensions<D1, D2>::type;
    using result_rep = std::common_type_t<R1, R2>;
    return quantity<result_dim, S, result_rep>(lhs.value() * rhs.value());
}

template <typename D1, typename D2, typename S, typename R1, typename R2>
constexpr auto operator/(const quantity<D1, S, R1>& lhs, const quantity<D2, S, R2>& rhs) {
    using result_dim = typename divide_dimensions<D1, D2>::type;
    using result_rep = std::common_type_t<R1, R2>;
    return quantity<result_dim, S, result_rep>(lhs.value() / rhs.value());
}

//==============================================================================
// R3.4: Scalar Operations
//==============================================================================

template <typename D, typename S, typename R, typename Scalar, std::enable_if_t<std::is_arithmetic_v<Scalar>, int> = 0>
constexpr quantity<D, S, std::common_type_t<R, Scalar>> operator*(const quantity<D, S, R>& q, const Scalar& s) {
    return quantity<D, S, std::common_type_t<R, Scalar>>(q.value() * s);
}

template <typename D, typename S, typename R, typename Scalar, std::enable_if_t<std::is_arithmetic_v<Scalar>, int> = 0>
constexpr quantity<D, S, std::common_type_t<R, Scalar>> operator*(const Scalar& s, const quantity<D, S, R>& q) {
    return q * s;
}

template <typename D, typename S, typename R, typename Scalar, std::enable_if_t<std::is_arithmetic_v<Scalar>, int> = 0>
constexpr quantity<D, S, std::common_type_t<R, Scalar>> operator/(const quantity<D, S, R>& q, const Scalar& s) {
    return quantity<D, S, std::common_type_t<R, Scalar>>(q.value() / s);
}

//==============================================================================
// R5.3: Comparison Operators
//==============================================================================

template <typename D, typename S, typename R>
constexpr bool operator==(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return lhs.value() == rhs.value();
}

template <typename D, typename S, typename R>
constexpr bool operator!=(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return !(lhs == rhs);
}

template <typename D, typename S, typename R>
constexpr bool operator<(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return lhs.value() < rhs.value();
}

template <typename D, typename S, typename R>
constexpr bool operator>(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return rhs < lhs;
}

template <typename D, typename S, typename R>
constexpr bool operator<=(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return !(rhs < lhs);
}

template <typename D, typename S, typename R>
constexpr bool operator>=(const quantity<D, S, R>& lhs, const quantity<D, S, R>& rhs) {
    return !(lhs < rhs);
}

//==============================================================================
// R4.1 & R4.2: Predefined SI Units (Dimensions and Quantity Aliases)
//==============================================================================

// Base Dimensions
using dimensionless_dim = dimension<0, 0, 0, 0, 0, 0, 0>;
using length_dim = dimension<1, 0, 0, 0, 0, 0, 0>;
using mass_dim = dimension<0, 1, 0, 0, 0, 0, 0>;
using time_dim = dimension<0, 0, 1, 0, 0, 0, 0>;
using current_dim = dimension<0, 0, 0, 1, 0, 0, 0>;

// Mechanical Derived Dimensions
using area_dim = dimension<2, 0, 0, 0, 0, 0, 0>;               // L^2
using volume_dim = dimension<3, 0, 0, 0, 0, 0, 0>;             // L^3
using velocity_dim = dimension<1, 0, -1, 0, 0, 0, 0>;          // L/T
using acceleration_dim = dimension<1, 0, -2, 0, 0, 0, 0>;      // L/T^2
using frequency_dim = dimension<0, 0, -1, 0, 0, 0, 0>;         // 1/T
using force_dim = dimension<1, 1, -2, 0, 0, 0, 0>;             // M*L/T^2
using torque_dim = dimension<2, 1, -2, 0, 0, 0, 0>;            // M*L^2/T^2
using moment_of_inertia_dim = dimension<2, 1, 0, 0, 0, 0, 0>;  // M*L^2

// Electrical Derived Dimensions
using charge_dim = dimension<0, 0, 1, 1, 0, 0, 0>;               // I*T
using voltage_dim = dimension<2, 1, -3, -1, 0, 0, 0>;            // M*L^2/(T^3*I)
using power_dim = dimension<2, 1, -3, 0, 0, 0, 0>;               // M*L^2/T^3
using resistance_dim = dimension<2, 1, -3, -2, 0, 0, 0>;         // M*L^2/(T^3*I^2)
using capacitance_dim = dimension<-2, -1, 4, 2, 0, 0, 0>;        // (T^4*I^2)/(M*L^2)
using inductance_dim = dimension<2, 1, -2, -2, 0, 0, 0>;         // (M*L^2)/(T^2*I^2)
using magnetic_flux_dim = dimension<2, 1, -2, -1, 0, 0, 0>;      // (M*L^2)/(T^2*I)
using mag_field_strength_dim = dimension<-1, 0, 0, 1, 0, 0, 0>;  // I/L
using flux_density_dim = dimension<0, 1, -2, -1, 0, 0, 0>;       // M/(T^2*I)

// SI Quantity Type Aliases
template <typename Rep = double>
using length = quantity<length_dim, si, Rep>;
template <typename Rep = double>
using mass = quantity<mass_dim, si, Rep>;
template <typename Rep = double>
using duration_type = quantity<time_dim, si, Rep>;
template <typename Rep = double>
using current = quantity<current_dim, si, Rep>;
template <typename Rep = double>
using velocity = quantity<velocity_dim, si, Rep>;
template <typename Rep = double>
using acceleration = quantity<acceleration_dim, si, Rep>;
template <typename Rep = double>
using frequency = quantity<frequency_dim, si, Rep>;
template <typename Rep = double>
using force = quantity<force_dim, si, Rep>;
template <typename Rep = double>
using torque = quantity<torque_dim, si, Rep>;
template <typename Rep = double>
using moment_of_inertia = quantity<moment_of_inertia_dim, si, Rep>;
template <typename Rep = double>
using angular_frequency = quantity<frequency_dim, si, Rep>;  // Radian is dimensionless
template <typename Rep = double>
using voltage = quantity<voltage_dim, si, Rep>;
template <typename Rep = double>
using power = quantity<power_dim, si, Rep>;
template <typename Rep = double>
using resistance = quantity<resistance_dim, si, Rep>;
template <typename Rep = double>
using impedance = quantity<resistance_dim, si, Rep>;  // Same dimension as resistance
template <typename Rep = double>
using capacitance = quantity<capacitance_dim, si, Rep>;
template <typename Rep = double>
using inductance = quantity<inductance_dim, si, Rep>;
template <typename Rep = double>
using magnetic_flux = quantity<magnetic_flux_dim, si, Rep>;
template <typename Rep = double>
using magnetic_field_strength = quantity<mag_field_strength_dim, si, Rep>;
template <typename Rep = double>
using magnetic_flux_density = quantity<flux_density_dim, si, Rep>;

//==============================================================================
// R4.3: Per-Unit Equivalents
//==============================================================================

template <typename Rep = double>
using voltage_pu = quantity<voltage_dim, per_unit, Rep>;
template <typename Rep = double>
using current_pu = quantity<current_dim, per_unit, Rep>;
template <typename Rep = double>
using power_pu = quantity<power_dim, per_unit, Rep>;
template <typename Rep = double>
using resistance_pu = quantity<resistance_dim, per_unit, Rep>;
template <typename Rep = double>
using impedance_pu = quantity<resistance_dim, per_unit, Rep>;  // Same dimension as resistance
template <typename Rep = double>
using frequency_pu = quantity<frequency_dim, per_unit, Rep>;
template <typename Rep = double>
using angular_frequency_pu = quantity<frequency_dim, per_unit, Rep>;
template <typename Rep = double>
using torque_pu = quantity<torque_dim, per_unit, Rep>;
template <typename Rep = double>
using inductance_pu = quantity<inductance_dim, per_unit, Rep>;
template <typename Rep = double>
using capacitance_pu = quantity<capacitance_dim, per_unit, Rep>;
template <typename Rep = double>
using magnetic_flux_pu = quantity<magnetic_flux_dim, per_unit, Rep>;
// ... other PU types can be defined similarly

//==============================================================================
// R2.4: Explicit System Conversion
//==============================================================================

// Convert from Per-Unit to SI
template <typename Dim, typename Rep1, typename Rep2>
constexpr auto to_si(const quantity<Dim, per_unit, Rep1>& pu_quantity, const quantity<Dim, si, Rep2>& base_quantity) {
    // The base quantity must have the same dimension as the per-unit quantity.
    // This check is implicitly handled by the template parameters.
    using result_rep = std::common_type_t<Rep1, Rep2>;
    return quantity<Dim, si, result_rep>(pu_quantity.value() * base_quantity.value());
}

// Convert from SI to Per-Unit
template <typename Dim, typename Rep1, typename Rep2>
constexpr auto to_pu(const quantity<Dim, si, Rep1>& si_quantity, const quantity<Dim, si, Rep2>& base_quantity) {
    // The base quantity must have the same dimension as the SI quantity.
    using result_rep = std::common_type_t<Rep1, Rep2>;
    return quantity<Dim, per_unit, result_rep>(si_quantity.value() / base_quantity.value());
}

//==============================================================================
// R5.2: Output Formatting
//==============================================================================

namespace internal {
template <typename Dim>
std::string get_unit_symbol() {
    if constexpr (std::is_same_v<Dim, dimensionless_dim>)
        return "";
    else if constexpr (std::is_same_v<Dim, length_dim>)
        return " m";
    else if constexpr (std::is_same_v<Dim, mass_dim>)
        return " kg";
    else if constexpr (std::is_same_v<Dim, time_dim>)
        return " s";
    else if constexpr (std::is_same_v<Dim, current_dim>)
        return " A";
    else if constexpr (std::is_same_v<Dim, velocity_dim>)
        return " m/s";
    else if constexpr (std::is_same_v<Dim, acceleration_dim>)
        return " m/s^2";
    else if constexpr (std::is_same_v<Dim, frequency_dim>)
        return " Hz";
    else if constexpr (std::is_same_v<Dim, force_dim>)
        return " N";
    else if constexpr (std::is_same_v<Dim, torque_dim>)
        return " Nm";
    else if constexpr (std::is_same_v<Dim, moment_of_inertia_dim>)
        return " kg*m^2";
    else if constexpr (std::is_same_v<Dim, voltage_dim>)
        return " V";
    else if constexpr (std::is_same_v<Dim, power_dim>)
        return " W";
    else if constexpr (std::is_same_v<Dim, resistance_dim>)
        return " Ohm";
    else if constexpr (std::is_same_v<Dim, capacitance_dim>)
        return " F";
    else if constexpr (std::is_same_v<Dim, inductance_dim>)
        return " H";
    else if constexpr (std::is_same_v<Dim, magnetic_flux_dim>)
        return " Wb";
    else if constexpr (std::is_same_v<Dim, mag_field_strength_dim>)
        return " A/m";
    else if constexpr (std::is_same_v<Dim, flux_density_dim>)
        return " T";
    else
        return " [unknown unit]";  // Fallback
}
}  // namespace internal

template <typename Dim, typename UnitSystem, typename Rep>
std::ostream& operator<<(std::ostream& os, const quantity<Dim, UnitSystem, Rep>& q) {
    os << q.value();
    if constexpr (std::is_same_v<UnitSystem, si>) {
        os << internal::get_unit_symbol<Dim>();
    } else if constexpr (std::is_same_v<UnitSystem, per_unit>) {
        os << " pu";
    }
    return os;
}

}  // namespace yeet::control