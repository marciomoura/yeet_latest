#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <numeric>

namespace yeet::control {

/**
 * @brief A 1D lookup table with linear interpolation using std::array for fixed-size, no-allocation storage.
 *
 * This class is designed for embedded systems where dynamic memory allocation is disallowed.
 * The size of the axis must be known at compile time.
 *
 * It stores a 1D grid of data points (y) defined over an independent axis (x).
 * It provides a method to query a value at any x coordinate, using linear
 * interpolation for points that fall between the grid lines.
 *
 * For points outside the defined grid, the value is clamped to the nearest endpoint.
 * Validation is performed via assert(), which has no overhead in release builds.
 *
 * @tparam T The floating-point type of the data (e.g., float, double).
 * @tparam NumX The number of points on the x-axis.
 */
template <typename T, size_t NumX>
class lookup_table_1d {
public:
    // Compile-time check for minimum dimension
    static_assert(NumX >= 2, "x-axis must have at least 2 points for interpolation.");

    lookup_table_1d() = default;

    /**
     * @brief Constructs and initializes the 1D lookup table.
     *
     * @param x_axis An array representing the breakpoints on the x-axis. Must be sorted.
     * @param y_values An array of data points corresponding to each x-axis breakpoint.
     */
    lookup_table_1d(const std::array<T, NumX>& x_axis, const std::array<T, NumX>& y_values)
        : _x_axis(x_axis), _y_values(y_values)
    {
        // Runtime check for sorted axis (in debug builds only)
        assert(std::is_sorted(_x_axis.begin(), _x_axis.end()) && "x-axis must be sorted.");
    }

    /**
     * @brief Configures the lookup table with a new axis and new values.
     *
     * @param x_axis New x-axis breakpoints.
     * @param y_values New data points.
     */
    void configure(const std::array<T, NumX>& x_axis, const std::array<T, NumX>& y_values)
    {
        assert(x_axis.size() == NumX && "x_axis size mismatch");
        assert(y_values.size() == NumX && "y_values size mismatch");

        _x_axis = x_axis;
        _y_values = y_values;

        // Runtime check for sorted axis (in debug builds only)
        assert(std::is_sorted(_x_axis.begin(), _x_axis.end()) && "x-axis must be sorted.");
    }

    /**
     * @brief Retrieves a value from the table for a given x coordinate.
     *
     * Performs linear interpolation if the point is within the grid.
     * Clamps the result to the endpoint if the point is outside the grid.
     *
     * @param x The coordinate on the x-axis.
     * @return The interpolated or clamped value.
     */
    [[nodiscard]] T get_value(T x) const
    {
        // --- Step 1: Find index and clamp coordinate ---
        size_t x_idx = find_lower_bound_index(_x_axis, x);

        // Clamp the input coordinate to the table boundaries
        x = std::clamp(x, _x_axis.front(), _x_axis.back());

        // --- Step 2: Get the two points for interpolation ---
        const T x0 = _x_axis[x_idx];
        const T x1 = _x_axis[x_idx + 1];

        const T y0 = _y_values[x_idx];
        const T y1 = _y_values[x_idx + 1];

        // --- Step 3: Calculate interpolation fraction ---
        T x_frac = (x1 - x0) > T{0} ? (x - x0) / (x1 - x0) : T{0};

        // --- Step 4: Perform linear interpolation ---
        T result = y0 + x_frac * (y1 - y0);

        return result;
    }

private:
    /**
     * @brief Finds the index of the lower bound for a value in a sorted axis array.
     */
    template <size_t N>
    [[nodiscard]] size_t find_lower_bound_index(const std::array<T, N>& axis, T value) const
    {
        auto it = std::lower_bound(axis.begin(), axis.end(), value);
        if (it == axis.begin()) return 0;
        if (it == axis.end()) return N - 2;
        return std::distance(axis.begin(), it) - 1;
    }

    std::array<T, NumX> _x_axis;
    std::array<T, NumX> _y_values;
};

}  // namespace yeet::control