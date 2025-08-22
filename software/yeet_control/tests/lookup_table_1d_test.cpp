#include "yeet_control/lookup_table_1d.hpp"

#include <gtest/gtest.h>

#include <array>

// Define constant for table dimension to make tests readable
constexpr size_t NumX = 4;

// Test fixture for creating a common lookup table object for tests
class LookupTable1DTest : public ::testing::Test {
protected:
    const std::array<float, NumX> x_axis{10.0f, 20.0f, 30.0f, 40.0f};
    const std::array<float, NumX> y_values{100.0f, 200.0f, 150.0f, 250.0f};
};

// Test that values on the exact grid points are retrieved correctly
TEST_F(LookupTable1DTest, HandlesExactGridPoints)
{
    yeet::control::lookup_table_1d<float, NumX> table(x_axis, y_values);
    EXPECT_FLOAT_EQ(table.get_value(10.0f), 100.0f);
    EXPECT_FLOAT_EQ(table.get_value(20.0f), 200.0f);
    EXPECT_FLOAT_EQ(table.get_value(30.0f), 150.0f);
    EXPECT_FLOAT_EQ(table.get_value(40.0f), 250.0f);
}

// Test that values are correctly interpolated when falling between grid points
TEST_F(LookupTable1DTest, PerformsLinearInterpolation)
{
    yeet::control::lookup_table_1d<float, NumX> table(x_axis, y_values);
    // Midpoint between 10 and 20 -> (100 + 200) / 2 = 150
    EXPECT_FLOAT_EQ(table.get_value(15.0f), 150.0f);
    // Midpoint between 20 and 30 -> (200 + 150) / 2 = 175
    EXPECT_FLOAT_EQ(table.get_value(25.0f), 175.0f);
    // Midpoint between 30 and 40 -> (150 + 250) / 2 = 200
    EXPECT_FLOAT_EQ(table.get_value(35.0f), 200.0f);
    // 25% of the way between 10 and 20 -> 100 + 0.25 * (200 - 100) = 125
    EXPECT_FLOAT_EQ(table.get_value(12.5f), 125.0f);
}

// Test that values outside the defined axis are clamped to the boundary
TEST_F(LookupTable1DTest, ClampsValuesOutsideBounds)
{
    yeet::control::lookup_table_1d<float, NumX> table(x_axis, y_values);
    // Below lower bound
    EXPECT_FLOAT_EQ(table.get_value(0.0f), 100.0f);
    EXPECT_FLOAT_EQ(table.get_value(5.0f), 100.0f);
    // Above upper bound
    EXPECT_FLOAT_EQ(table.get_value(50.0f), 250.0f);
    EXPECT_FLOAT_EQ(table.get_value(100.0f), 250.0f);
}

// Test default constructor and configure method
TEST_F(LookupTable1DTest, ConfigureMethodWorks)
{
    yeet::control::lookup_table_1d<float, NumX> table;  // Default construct
    table.configure(x_axis, y_values);                  // Configure it

    EXPECT_FLOAT_EQ(table.get_value(15.0f), 150.0f);
    EXPECT_FLOAT_EQ(table.get_value(50.0f), 250.0f);

    // Re-configure with new data
    const std::array<float, NumX> new_x_axis{0.0f, 1.0f, 2.0f, 3.0f};
    const std::array<float, NumX> new_y_values{0.0f, -10.0f, -20.0f, -30.0f};
    table.configure(new_x_axis, new_y_values);

    EXPECT_FLOAT_EQ(table.get_value(0.5f), -5.0f);
    EXPECT_FLOAT_EQ(table.get_value(2.5f), -25.0f);
    EXPECT_FLOAT_EQ(table.get_value(-1.0f), 0.0f);  // Clamping with new data
}