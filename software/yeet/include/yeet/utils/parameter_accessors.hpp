#pragma once

#include "yeet/parameter.hpp"
#include "yeet/utils/low_level_access.hpp"
#include "yeet_control/angle_wrapped.hpp"  // For angle_wrapped and angle_t

/**
 * @file parameter_accessors.hpp
 * @brief Reusable parameter accessor factories to eliminate lambda boilerplate in datamodels
 *
 * This header provides template factory functions that create low_level_access objects
 * for all parameter types without requiring inline lambda definitions. The generic
 * make_units_accessor template works with any unit type that supports direct float conversion.
 *
 * Example usage (before):
 *     auto& freq_param = _model_parameters->frequency;
 *     entry.parameters.insert("frequency",
 *         low_level_access(
 *             [&freq_param]() -> float { return static_cast<float>(freq_param.read()); },
 *             [&freq_param](float value_hz) { freq_param.write(frequency_t{value_hz}); }
 *         )
 *     );
 *
 * Example usage (after):
 *     entry.parameters.insert("frequency", yeet::utils::make_units_accessor(_model_parameters->frequency));
 *
 * For advanced custom conversions, use make_parameter_accessor with explicit lambdas.
 */

#include "yeet/parameter.hpp"
#include "yeet/utils/low_level_access.hpp"

namespace yeet::utils {

/**
 * @brief Generic template for creating low_level_access for any parameter type with custom conversions
 *
 * @tparam ParamT Parameter type (e.g., yeet::parameter<T>)
 * @tparam ToFloatFn Function/functor to convert ParamT::read() to float
 * @tparam FromFloatFn Function/functor to convert float to ParamT::write()
 * @param param Reference to the parameter object
 * @param to_float Conversion function: ParamT::read() -> float
 * @param from_float Conversion function: float -> ParamT::write()
 * @return low_level_access configured for generic type
 */
template <typename ParamT, typename ToFloatFn, typename FromFloatFn>
inline low_level_access make_parameter_accessor(ParamT& param, ToFloatFn to_float, FromFloatFn from_float) noexcept
{
    return low_level_access([&param, to_float]() -> float { return to_float(param.read()); },
        [&param, from_float](float value) { param.write(from_float(value)); });
}

/**
 * @brief Generic template for creating low_level_access for unit types
 *
 * Works with any unit type (frequency_t, duration_t, angle_wrapped, voltage_pu_t, etc.)
 * that supports implicit float conversion via operator float or similar.
 *
 * Special handling:
 * - duration_t: Reads value in seconds via .value() method
 * - angle_wrapped: Reads value in radians via .get_radians() method
 * - Other units: Use direct float conversion
 *
 * @tparam UnitT The unit type (frequency_t, duration_t, angle_wrapped, etc.)
 * @param param Reference to the parameter<UnitT> object
 * @return low_level_access configured for the unit type
 */
template <typename UnitT>
inline low_level_access make_units_accessor(parameter<UnitT>& param) noexcept
{
    return make_parameter_accessor(
        param, [](const UnitT& v) -> float { return static_cast<float>(v); },
        [](float f) -> UnitT { return static_cast<UnitT>(f); });
}

/**
 * @brief Creates low_level_access for boolean parameters
 *
 * Converts between bool and float using standard conventions:
 * - bool to float: true -> 1.0f, false -> 0.0f
 * - float to bool: value != 0.0f (non-zero is true, zero is false)
 *
 * @param param Reference to the parameter<bool> object
 * @return low_level_access configured for boolean type
 */
inline low_level_access make_bool_accessor(parameter<bool>& param) noexcept
{
    return make_parameter_accessor(
        param, [](bool v) -> float { return v ? 1.0f : 0.0f; },
        [](float f) -> bool { return f > 0.0f ? true : false; });
}

/**
 * @brief Creates low_level_access for enumeration parameters
 *
 * Converts between enum and float using integer casting:
 * - enum to float: static_cast<int>(enum_value) -> float
 * - float to enum: static_cast<EnumT>(static_cast<int>(float_value))
 *
 * This template works with any enumeration type and eliminates the need for
 * manual lambda definitions.
 *
 * @tparam EnumT The enumeration type
 * @param param Reference to the parameter<EnumT> object
 * @return low_level_access configured for enumeration type
 *
 * @example
 *     parameter<my_enum> enum_param;
 *     entry.parameters.insert("mode", make_enum_accessor(enum_param));
 */
template <typename EnumT>
inline low_level_access make_enum_accessor(parameter<EnumT>& param) noexcept
{
    return make_parameter_accessor(
        param, [](const EnumT& v) -> float { return static_cast<float>(static_cast<int>(v)); },
        [](float f) -> EnumT { return static_cast<EnumT>(static_cast<int>(f)); });
}

}  // namespace yeet::utils

namespace yeet::utils {

/**
 * @brief Creates low_level_access for angle_wrapped parameters (radians interface)
 *
 * Converts between angle_wrapped and float (radians):
 * - angle_wrapped to float: extract radians via get_radians()
 * - float to angle_wrapped: construct from radians via from_radians()
 *
 * This specialized accessor is necessary because angle_wrapped does not have an
 * implicit float conversion operator.
 *
 * @param param Reference to the parameter<yeet::angle_wrapped> object
 * @return low_level_access configured for angle_wrapped type (radians)
 */
inline low_level_access make_angle_wrapped_accessor(parameter<yeet::angle_wrapped>& param) noexcept
{
    return low_level_access([&param]() -> float { return static_cast<float>(param.read().get_radians()); },
        [&param](float value_rad) { param.write(yeet::angle_wrapped::from_radians(yeet::angle_t{value_rad})); });
}

/**
 * @brief Creates low_level_access for angle_wrapped parameters (degrees interface)
 *
 * Converts between angle_wrapped and float (degrees):
 * - angle_wrapped to float: extract degrees via get_degrees()
 * - float to angle_wrapped: construct from degrees via from_degrees()
 *
 * This accessor provides a user-friendly degrees interface while maintaining
 * internal representation in radians. Useful for configuration parameters where
 * degrees are the preferred engineering unit.
 *
 * @param param Reference to the parameter<yeet::angle_wrapped> object
 * @return low_level_access configured for angle_wrapped type (degrees)
 */
inline low_level_access make_degrees_angle_wrapped_accessor(parameter<yeet::angle_wrapped>& param) noexcept
{
    return low_level_access([&param]() -> float { return static_cast<float>(param.read().get_degrees()); },
        [&param](float value_deg) { param.write(yeet::angle_wrapped::from_degrees(value_deg)); });
}

/**
 * @brief Creates low_level_access for a single element in an array parameter
 *
 * This template enables per-element access to std::array<T, N> parameters for datamodel registration.
 * The element type must support conversion to/from float.
 *
 * @tparam ElementT The element type stored in the array (e.g., speed_pu_t, torque_pu_t)
 * @tparam N The size of the array
 * @param param Reference to the parameter<std::array<ElementT, N>> object
 * @param index The array index to access (0-based)
 * @return low_level_access configured for the specific array element
 *
 * @example
 *     parameter<std::array<speed_pu_t, 7>> speed_axis;
 *     // Per-element registration for array[0]:
 *     entry.parameters.insert("speed_axis[0]", make_array_element_accessor(speed_axis, 0));
 */
template <typename ElementT, std::size_t N>
inline low_level_access make_array_element_accessor(
    parameter<std::array<ElementT, N>>& param, std::size_t index) noexcept
{
    return low_level_access([&param, index]() -> float { return static_cast<float>(param.read()[index].value()); },
        [&param, index](float value) {
            auto table = param.read();
            table[index] = ElementT(value);
            param.write(table);
        });
}

}  // namespace yeet::utils
