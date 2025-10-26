#pragma once

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
        [](float f) -> UnitT { return UnitT{f}; });
}

}  // namespace yeet::utils
