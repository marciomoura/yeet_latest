#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <numeric>

namespace yeet::control {

/**
 * @brief A 2D lookup table with bilinear interpolation using std::array for fixed-size, no-allocation storage.
 *
 * This class is designed for embedded systems where dynamic memory allocation is disallowed.
 * The sizes of the axes must be known at compile time.
 *
 * It stores a 2D grid of data points (z) defined over two independent axes (x and y).
 * It provides a method to query a value at any (x, y) coordinate, using bilinear
 * interpolation for points that fall between the grid lines.
 *
 * For points outside the defined grid, the value is clamped to the nearest edge/corner.
 * Validation is performed via assert(), which has no overhead in release builds.
 *
 * @tparam T The floating-point type of the data (e.g., float, double).
 * @tparam NumX The number of points on the x-axis.
 * @tparam NumY The number of points on the y-axis.
 */
template <typename T, size_t NumX, size_t NumY>
class lookup_table_2d {
public:
    // Compile-time checks for minimum dimensions
    static_assert(NumX >= 2, "x-axis must have at least 2 points for interpolation.");
    static_assert(NumY >= 2, "y-axis must have at least 2 points for interpolation.");

    lookup_table_2d() = default;

    /**
     * @brief Constructs and initializes the 2D lookup table.
     *
     * @param x_axis An array representing the breakpoints on the x-axis. Must be sorted.
     * @param y_axis An array representing the breakpoints on the y-axis. Must be sorted.
     * @param z_values A 2D array of data points. Dimensions are enforced at compile time.
     */
    lookup_table_2d(const std::array<T, NumX>& x_axis, const std::array<T, NumY>& y_axis,
                    const std::array<std::array<T, NumX>, NumY>& z_values)
        : _x_axis(x_axis), _y_axis(y_axis), _z_values(z_values) {
        // Runtime check for sorted axes (in debug builds only)
        assert(std::is_sorted(_x_axis.begin(), _x_axis.end()) && "x-axis must be sorted.");
        assert(std::is_sorted(_y_axis.begin(), _y_axis.end()) && "y-axis must be sorted.");
    }

    /**
     * @brief Configures the lookup table with new axes and values.
     *
     * @param x_axis New x-axis breakpoints.
     * @param y_axis New y-axis breakpoints.
     * @param z_values New 2D data points.
     */
    void configure(const std::array<T, NumX>& x_axis, const std::array<T, NumY>& y_axis,
                   const std::array<std::array<T, NumX>, NumY>& z_values) {
        assert(x_axis.size() == NumX && "x_axis size mismatch");
        assert(y_axis.size() == NumY && "y_axis size mismatch");
        assert(z_values.size() == NumY && "z_values row count mismatch");
        for (const auto& row : z_values) {
            assert(row.size() == NumX && "z_values column count mismatch");
        }

        _x_axis = x_axis;
        _y_axis = y_axis;
        _z_values = z_values;

        // Runtime check for sorted axes (in debug builds only)
        assert(std::is_sorted(_x_axis.begin(), _x_axis.end()) && "x-axis must be sorted.");
        assert(std::is_sorted(_y_axis.begin(), _y_axis.end()) && "y-axis must be sorted.");
    }

    /**
     * @brief Retrieves a value from the table for a given (x, y) coordinate.
     *
     * Performs bilinear interpolation if the point is within the grid.
     * Clamps the result to the edge if the point is outside the grid.
     *
     * @param x The coordinate on the x-axis.
     * @param y The coordinate on the y-axis.
     * @return The interpolated or clamped value.
     */
    [[nodiscard]] T get_value(T x, T y) const {
        // --- Step 1: Find indices and clamp coordinates ---
        size_t x_idx = find_lower_bound_index(_x_axis, x);
        size_t y_idx = find_lower_bound_index(_y_axis, y);

        // Clamp the input coordinates to the table boundaries
        x = std::clamp(x, _x_axis.front(), _x_axis.back());
        y = std::clamp(y, _y_axis.front(), _y_axis.back());

        // --- Step 2: Get the four corner points of the interpolation cell ---
        const T x0 = _x_axis[x_idx];
        const T x1 = _x_axis[x_idx + 1];
        const T y0 = _y_axis[y_idx];
        const T y1 = _y_axis[y_idx + 1];

        const T z00 = _z_values[y_idx][x_idx];
        const T z10 = _z_values[y_idx][x_idx + 1];
        const T z01 = _z_values[y_idx + 1][x_idx];
        const T z11 = _z_values[y_idx + 1][x_idx + 1];

        // --- Step 3: Calculate interpolation fractions ---
        T x_frac = (x1 - x0) > T{0} ? (x - x0) / (x1 - x0) : T{0};
        T y_frac = (y1 - y0) > T{0} ? (y - y0) / (y1 - y0) : T{0};

        // --- Step 4: Perform bilinear interpolation ---
        T z_interp_y0 = z00 + x_frac * (z10 - z00);
        T z_interp_y1 = z01 + x_frac * (z11 - z01);
        T result = z_interp_y0 + y_frac * (z_interp_y1 - z_interp_y0);

        return result;
    }

private:
    /**
     * @brief Finds the index of the lower bound for a value in a sorted axis array.
     */
    template <size_t N>
    [[nodiscard]] size_t find_lower_bound_index(const std::array<T, N>& axis, T value) const {
        auto it = std::lower_bound(axis.begin(), axis.end(), value);
        if (it == axis.begin()) return 0;
        if (it == axis.end()) return N - 2;
        return std::distance(axis.begin(), it) - 1;
    }

    std::array<T, NumX> _x_axis;
    std::array<T, NumY> _y_axis;
    std::array<std::array<T, NumX>, NumY> _z_values;
};

}  // namespace yeet::control
