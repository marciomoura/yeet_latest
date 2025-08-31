#include "sfc/grid_side_hardware_mock.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc::mock;
using yeet::duration_t;
using yeet::frequency_t;
using yeet::real_t;

class GridSideMeasurementsMockTest : public ::testing::Test {
protected:
    static constexpr duration_t sampling_period{0.001};
    grid_side_hardware_mock meas_mock{sampling_period};
};

TEST_F(GridSideMeasurementsMockTest, InitialStateIsZero)
{
    EXPECT_EQ(meas_mock.get_v_ab_counts().get_counts(), 0);
    EXPECT_EQ(meas_mock.get_v_bc_counts().get_counts(), 0);
    EXPECT_EQ(meas_mock.get_i_a_counts().get_counts(), 0);
    EXPECT_EQ(meas_mock.get_i_c_counts().get_counts(), 0);
}

TEST_F(GridSideMeasurementsMockTest, ResetReturnsToZero)
{
    meas_mock.configure_voltage_adc(1.0, frequency_t{100.0}, 12, 5.0);
    meas_mock.configure_current_adc(1.0, frequency_t{100.0}, 12, 5.0);
    meas_mock.update(1.0, 1.0, 1.0, 1.0);
    ASSERT_NE(meas_mock.get_v_ab_counts().get_counts(), 0);

    meas_mock.reset();
    EXPECT_EQ(meas_mock.get_v_ab_counts().get_counts(), 0);
    EXPECT_EQ(meas_mock.get_v_bc_counts().get_counts(), 0);
    EXPECT_EQ(meas_mock.get_i_a_counts().get_counts(), 0);
    EXPECT_EQ(meas_mock.get_i_c_counts().get_counts(), 0);
}

// TEST_F(GridSideMeasurementsMockTest, UpdatePropagatesToOutputs)
// {
//     meas_mock.configure_voltage_adc(duration_t{1.0}, frequency_t{499.0}, 12, 5.0);
//     meas_mock.configure_current_adc(duration_t{2.0}, frequency_t{499.0}, 16, 10.0);

//     // Settle the filter
//     for (int i = 0; i < 100; ++i) {
//         meas_mock.update(2.5, -2.5, 1.0, -1.0);
//     }

//     // Check voltage
//     const std::int32_t expected_v_counts = (1 << 11) - 1;  // Max for 12-bit signed
//     EXPECT_NEAR(meas_mock.get_v_ab_counts().get_counts(), expected_v_counts / 2.0, 1);
//     EXPECT_NEAR(meas_mock.get_v_bc_counts().get_counts(), -expected_v_counts / 2.0, 1);

//     // Check current (gain is 2.0, so 1.0A -> 2.0V)
//     const std::int32_t expected_i_counts = (1 << 15) - 1;  // Max for 16-bit signed
//     EXPECT_NEAR(
//         meas_mock.get_i_a_counts().get_counts(), static_cast<std::int32_t>(expected_i_counts * (2.0 / 10.0)), 1);
//     EXPECT_NEAR(
//         meas_mock.get_i_c_counts().get_counts(), static_cast<std::int32_t>(expected_i_counts * (-2.0 / 10.0)), 1);
// }