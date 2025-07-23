#include "yeet_control/lookup_table_2d.hpp"

#include <gtest/gtest.h>

#include <array>

// Define constants for table dimensions to make tests readable
constexpr size_t NumX = 3;
constexpr size_t NumY = 4;

// Test fixture for creating a common lookup table object for tests
class LookupTable2DTest : public ::testing::Test {
protected:
    // The test data is now stored in std::array
    const std::array<float, NumX> x_axis{10.0f, 20.0f, 30.0f};
    const std::array<float, NumY> y_axis{2.0f, 4.0f, 6.0f, 8.0f};

    const std::array<std::array<float, NumX>, NumY> z_values{{
        {14.0f, 24.0f, 34.0f},  // y=2.0
        {18.0f, 28.0f, 38.0f},  // y=4.0
        {22.0f, 32.0f, 42.0f},  // y=6.0
        {26.0f, 36.0f, 46.0f}   // y=8.0
    }};
};

// Test that values on the exact grid points are retrieved correctly
TEST_F(LookupTable2DTest, HandlesExactGridPoints) {
    yeet::control::lookup_table_2d<float, NumX, NumY> table(x_axis, y_axis, z_values);
    EXPECT_FLOAT_EQ(table.get_value(10.0f, 2.0f), 14.0f);
    EXPECT_FLOAT_EQ(table.get_value(30.0f, 8.0f), 46.0f);
    EXPECT_FLOAT_EQ(table.get_value(20.0f, 6.0f), 32.0f);
}

// Test that values are correctly interpolated when falling between grid points
TEST_F(LookupTable2DTest, PerformsBilinearInterpolation) {
    yeet::control::lookup_table_2d<float, NumX, NumY> table(x_axis, y_axis, z_values);
    EXPECT_FLOAT_EQ(table.get_value(15.0f, 3.0f), 21.0f);
    EXPECT_FLOAT_EQ(table.get_value(25.0f, 7.0f), 39.0f);
}

// Test interpolation along a single axis
TEST_F(LookupTable2DTest, InterpolatesAlongSingleAxis) {
    yeet::control::lookup_table_2d<float, NumX, NumY> table(x_axis, y_axis, z_values);
    EXPECT_FLOAT_EQ(table.get_value(10.0f, 5.0f), 20.0f);
    EXPECT_FLOAT_EQ(table.get_value(25.0f, 2.0f), 29.0f);
}

// Test that values outside the defined axes are clamped to the boundary
TEST_F(LookupTable2DTest, ClampsValuesOutsideBounds) {
    yeet::control::lookup_table_2d<float, NumX, NumY> table(x_axis, y_axis, z_values);
    EXPECT_FLOAT_EQ(table.get_value(5.0f, 3.0f), 16.0f);
    EXPECT_FLOAT_EQ(table.get_value(35.0f, 5.0f), 40.0f);
    EXPECT_FLOAT_EQ(table.get_value(15.0f, 1.0f), 19.0f);
    EXPECT_FLOAT_EQ(table.get_value(25.0f, 9.0f), 41.0f);
}

// Test clamping to the four corners of the grid
TEST_F(LookupTable2DTest, ClampsToCorners) {
    yeet::control::lookup_table_2d<float, NumX, NumY> table(x_axis, y_axis, z_values);
    EXPECT_FLOAT_EQ(table.get_value(0.0f, 0.0f), 14.0f);
    EXPECT_FLOAT_EQ(table.get_value(40.0f, 10.0f), 46.0f);
    EXPECT_FLOAT_EQ(table.get_value(5.0f, 10.0f), 26.0f);
    EXPECT_FLOAT_EQ(table.get_value(35.0f, 1.0f), 34.0f);
}

// --- Compile-Time Validation Examples ---
// The tests for construction with invalid dimensions have been removed because
// they are now caught at compile time, which is safer and more efficient.
// To test this, you can uncomment the code below. It should fail to compile.

/*
TEST(LookupTable2DValidationTest, FailsToCompileWithInvalidDimensions) {
    // ERROR: Mismatched X dimension (2 instead of 3)
    // std::array<float, 2> bad_x_axis = {10.0f, 20.0f};
    // yeet::control::lookup_table_2d<float, 2, NumY> bad_table(bad_x_axis, y_axis, z_values);

    // ERROR: Axis size less than 2
    // std::array<float, 1> small_x_axis = {10.0f};
    // std::array<float, 4> y_axis_ok = {1,2,3,4};
    // std::array<std::array<float, 1>, 4> z_values_ok = {{{{1}},{{2}},{{3}},{{4}}}};
    // yeet::control::lookup_table_2d<float, 1, 4> bad_table(small_x_axis, y_axis_ok, z_values_ok);
}
*/
