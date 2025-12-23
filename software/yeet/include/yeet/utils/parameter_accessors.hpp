#pragma once

#include <array>

#include "yeet/parameter.hpp"
#include "yeet/utils/low_level_access.hpp"
#include "yeet_control/angle_wrapped.hpp"  // For angle_wrapped and angle_t

/**
 * @file parameter_accessors.hpp
 * @brief Reusable parameter accessor factories using function pointers (zero heap allocation)
 *
 * This header provides template factory functions that create low_level_access objects
 * using function pointers instead of std::function. This eliminates heap allocations
 * and reduces memory footprint from 64-128 bytes to 24 bytes per accessor.
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
 * Memory optimization:
 * - Old approach (std::function): 64-128 bytes per accessor
 * - New approach (function pointers): 24 bytes per accessor
 * - Savings: 40-104 bytes per parameter, 8-20KB for 196 parameters
 */

namespace yeet::utils {

// Internal implementation details
namespace detail {

// Generic read/write function pointers for unit types
template <typename UnitT>
static float read_units(void* context) noexcept
{
    auto* param = static_cast<parameter<UnitT>*>(context);
    return static_cast<float>(param->read());
}

template <typename UnitT>
static void write_units(void* context, float value) noexcept
{
    auto* param = static_cast<parameter<UnitT>*>(context);
    param->write(static_cast<UnitT>(value));
}

// Boolean read/write function pointers
static float read_bool(void* context) noexcept
{
    auto* param = static_cast<parameter<bool>*>(context);
    return param->read() ? 1.0f : 0.0f;
}

static void write_bool(void* context, float value) noexcept
{
    auto* param = static_cast<parameter<bool>*>(context);
    param->write(value > 0.0f);
}

// Enum read/write function pointers
template <typename EnumT>
static float read_enum(void* context) noexcept
{
    auto* param = static_cast<parameter<EnumT>*>(context);
    return static_cast<float>(static_cast<int>(param->read()));
}

template <typename EnumT>
static void write_enum(void* context, float value) noexcept
{
    auto* param = static_cast<parameter<EnumT>*>(context);
    param->write(static_cast<EnumT>(static_cast<int>(value)));
}

// Angle wrapped read/write (radians)
static float read_angle_radians(void* context) noexcept
{
    auto* param = static_cast<parameter<yeet::angle_wrapped>*>(context);
    return static_cast<float>(param->read().get_radians());
}

static void write_angle_radians(void* context, float value) noexcept
{
    auto* param = static_cast<parameter<yeet::angle_wrapped>*>(context);
    param->write(yeet::angle_wrapped::from_radians(yeet::angle_t{value}));
}

// Angle wrapped read/write (degrees)
static float read_angle_degrees(void* context) noexcept
{
    auto* param = static_cast<parameter<yeet::angle_wrapped>*>(context);
    return static_cast<float>(param->read().get_degrees());
}

static void write_angle_degrees(void* context, float value) noexcept
{
    auto* param = static_cast<parameter<yeet::angle_wrapped>*>(context);
    param->write(yeet::angle_wrapped::from_degrees(value));
}

// Array element context structure
template <typename ElementT, std::size_t N>
struct array_element_context {
    parameter<::std::array<ElementT, N>>* param;
    ::std::size_t index;
};

// Static storage for array element contexts (max 64 array accessors)
template <typename ElementT, std::size_t N>
struct array_accessor_storage {
    static constexpr ::std::size_t MAX_ARRAY_ACCESSORS = 64;
    static ::std::array<array_element_context<ElementT, N>, MAX_ARRAY_ACCESSORS> contexts;
    static ::std::size_t next_index;

    static array_element_context<ElementT, N>* allocate(
        parameter<::std::array<ElementT, N>>* param, ::std::size_t index)
    {
        if (next_index >= MAX_ARRAY_ACCESSORS) {
            return nullptr;  // Out of slots - should not happen in practice
        }
        contexts[next_index] = {param, index};
        return &contexts[next_index++];
    }
};

template <typename ElementT, std::size_t N>
::std::array<array_element_context<ElementT, N>, array_accessor_storage<ElementT, N>::MAX_ARRAY_ACCESSORS>
    array_accessor_storage<ElementT, N>::contexts = {};

template <typename ElementT, std::size_t N>
::std::size_t array_accessor_storage<ElementT, N>::next_index = 0;

// Array element read/write function pointers
template <typename ElementT, std::size_t N>
static float read_array_element(void* context) noexcept
{
    auto* ctx = static_cast<array_element_context<ElementT, N>*>(context);
    return static_cast<float>(ctx->param->read()[ctx->index].value());
}

template <typename ElementT, std::size_t N>
static void write_array_element(void* context, float value) noexcept
{
    auto* ctx = static_cast<array_element_context<ElementT, N>*>(context);
    auto table = ctx->param->read();
    table[ctx->index] = ElementT(value);
    ctx->param->write(table);
}

}  // namespace detail

/**
 * @brief Generic template for creating low_level_access for unit types
 *
 * Works with any unit type (frequency_t, duration_t, voltage_pu_t, etc.)
 * that supports implicit float conversion via operator float or similar.
 *
 * @tparam UnitT The unit type (frequency_t, duration_t, etc.)
 * @param param Reference to the parameter<UnitT> object
 * @return low_level_access configured for the unit type
 */
template <typename UnitT>
inline low_level_access make_units_accessor(parameter<UnitT>& param) noexcept
{
    return low_level_access(detail::read_units<UnitT>, detail::write_units<UnitT>, &param);
}

/**
 * @brief Creates low_level_access for boolean parameters
 *
 * Converts between bool and float using standard conventions:
 * - bool to float: true -> 1.0f, false -> 0.0f
 * - float to bool: value > 0.0f (non-zero is true, zero/negative is false)
 *
 * @param param Reference to the parameter<bool> object
 * @return low_level_access configured for boolean type
 */
inline low_level_access make_bool_accessor(parameter<bool>& param) noexcept
{
    return low_level_access(detail::read_bool, detail::write_bool, &param);
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
    return low_level_access(detail::read_enum<EnumT>, detail::write_enum<EnumT>, &param);
}

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
    return low_level_access(detail::read_angle_radians, detail::write_angle_radians, &param);
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
    return low_level_access(detail::read_angle_degrees, detail::write_angle_degrees, &param);
}

/**
 * @brief Creates low_level_access for a single element in an array parameter
 *
 * This template enables per-element access to std::array<T, N> parameters for datamodel registration.
 * The element type must support conversion to/from float via .value() method.
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
    auto* ctx = detail::array_accessor_storage<ElementT, N>::allocate(&param, index);
    return low_level_access(detail::read_array_element<ElementT, N>, detail::write_array_element<ElementT, N>, ctx);
}

}  // namespace yeet::utils
