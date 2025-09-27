#pragma once

#include <cstdint>

#include "units.hpp"

namespace yeet::control {

/**
 * @class adc_value_t
 * @brief Represents an ADC value with embedded conversion parameters.
 *
 * This class stores an ADC count value along with the conversion parameters
 * needed to convert between digital counts and real-world values. It provides
 * methods to convert to/from real values and perform arithmetic operations.
 */
class adc_value_t {
public:
    /**
     * @brief Default constructor - creates an ADC value with zero counts and default conversion.
     */
    adc_value_t() = default;

    /**
     * @brief Constructs an ADC value with conversion parameters.
     * @param bits The number of bits of the ADC (e.g., 12, 16).
     * @param full_scale_value The full-scale value of the ADC's bipolar range [-V, +V].
     * @param counts Initial count value (default: 0).
     */
    adc_value_t(int bits, real_t full_scale_value, std::int32_t counts = 0);

    /**
     * @brief Constructs an ADC value from a real value using given conversion parameters.
     * @param bits The number of bits of the ADC (e.g., 12, 16).
     * @param full_scale_value The full-scale value of the ADC's bipolar range [-V, +V].
     * @param real_value The real-world value to convert to counts.
     */
    static adc_value_t from_real(int bits, real_t full_scale_value, real_t real_value);

    /**
     * @brief Sets the count value directly.
     * @param counts The new count value.
     */
    void set_counts(std::int32_t counts) noexcept;

    /**
     * @brief Sets the value from a real-world value using the configured conversion.
     * @param real_value The real-world value to convert and store.
     */
    void set_real(real_t real_value) noexcept;

    /**
     * @brief Gets the raw count value.
     * @return The ADC count value.
     */
    std::int32_t get_counts() const noexcept;

    /**
     * @brief Converts the count value to a real-world value.
     * @return The real-world value.
     */
    real_t to_real() const noexcept;

    /**
     * @brief Implicit conversion to real_t for convenience.
     */
    operator real_t() const noexcept;

    /**
     * @brief Assignment from count value.
     */
    adc_value_t& operator=(std::int32_t counts) noexcept;

    /**
     * @brief Assignment from real value.
     */
    adc_value_t& operator=(real_t real_value) noexcept;

private:
    std::int32_t _counts{0};
    real_t _counts_to_real_factor{1.0};
    real_t _real_to_counts_factor{1.0};
    std::int32_t _max_counts{2047};  // Default for 12-bit ADC

    void _calculate_factors(int bits, real_t full_scale_value);
};

}  // namespace yeet::control

// New lightweight ADC abstraction components
namespace yeet::control {

/**
 * @brief Plain container for raw ADC sample counts (signed bipolar representation).
 */
struct adc_counts {
    std::int32_t value{0};
    adc_counts() = default;
    explicit adc_counts(std::int32_t v) : value(v) {}
};

/**
 * @brief Converter holding scaling factors (bits & full-scale) to map counts <-> real.
 *
 * Configure once, then use to convert any number of samples. All operations are
 * noexcept and allocation free.
 */
class adc_converter {
public:
    adc_converter() = default;

    adc_converter(int bits, real_t full_scale) { configure(bits, full_scale); }

    void configure(int bits, real_t full_scale) noexcept
    {
        // Basic asserts left to debug builds; in release we still guard divisions.
        if (bits <= 1) bits = 2;
        if (full_scale <= real_t{0}) full_scale = real_t{1};
        _bits = bits;
        _full_scale = full_scale;
        _max_counts = (1 << (bits - 1)) - 1;  // e.g., 2047 for 12-bit
        _counts_to_real = _full_scale / static_cast<real_t>(_max_counts);
        _real_to_counts = static_cast<real_t>(_max_counts) / _full_scale;
    }

    [[nodiscard]] adc_counts to_counts(real_t real_value) const noexcept
    {
        auto raw = static_cast<std::int32_t>(std::llround(real_value * _real_to_counts));
        if (raw > _max_counts)
            raw = _max_counts;
        else if (raw < -_max_counts - 1)
            raw = -_max_counts - 1;  // symmetric two's complement range
        return adc_counts{raw};
    }

    [[nodiscard]] real_t to_real(adc_counts counts) const noexcept
    {
        return static_cast<real_t>(counts.value) * _counts_to_real;
    }

    [[nodiscard]] int bits() const noexcept { return _bits; }
    [[nodiscard]] real_t full_scale() const noexcept { return _full_scale; }
    [[nodiscard]] std::int32_t max_counts() const noexcept { return _max_counts; }

private:
    int _bits{12};
    real_t _full_scale{1.0};
    std::int32_t _max_counts{2047};
    real_t _counts_to_real{1.0 / 2047.0};
    real_t _real_to_counts{2047.0};
};

}  // namespace yeet::control