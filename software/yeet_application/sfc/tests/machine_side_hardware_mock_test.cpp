// #include "sfc/machine_side_hardware_mock.hpp"

// #include <gtest/gtest.h>

// using namespace yeet::application::sfc::mock;
// using yeet::duration_t;
// using yeet::frequency_t;
// using yeet::real_t;

// class MachineSideHardwareMockTest : public ::testing::Test {
// protected:
//     static constexpr duration_t sampling_period{0.001};
//     machine_side_hardware_mock meas_mock{sampling_period};
// };

// TEST_F(MachineSideHardwareMockTest, InitialStateIsZero)
// {
//     EXPECT_EQ(meas_mock.get_v_ab_counts().get_counts(), 0);
//     EXPECT_EQ(meas_mock.get_v_bc_counts().get_counts(), 0);
// }

// TEST_F(MachineSideHardwareMockTest, ResetReturnsToZero)
// {
//     meas_mock.configure_voltage_adc(1.0, frequency_t{100.0}, 12, 5.0);
//     meas_mock.update(1.0, 1.0);
//     ASSERT_NE(meas_mock.get_v_ab_counts().get_counts(), 0);

//     meas_mock.reset();
//     EXPECT_EQ(meas_mock.get_v_ab_counts().get_counts(), 0);
//     EXPECT_EQ(meas_mock.get_v_bc_counts().get_counts(), 0);
// }

// TEST_F(MachineSideHardwareMockTest, UpdatePropagatesToOutputs)
// {
//     meas_mock.configure_voltage_adc(1.0, frequency_t{499.0}, 12, 5.0);

//     // Settle the filter by running update multiple times
//     for (int i = 0; i < 100; ++i) {
//         meas_mock.update(2.5, -2.5);
//     }

//     // Calculate expected counts for a 12-bit signed ADC
//     // Input of 2.5V on a +/-5.0V scale should be half of the max positive counts.
//     const std::int32_t max_counts = (1 << 11) - 1;  // 2047 for 12-bit signed
//     EXPECT_NEAR(meas_mock.get_v_ab_counts().get_counts(), max_counts / 2.0, 1);
//     EXPECT_NEAR(meas_mock.get_v_bc_counts().get_counts(), -max_counts / 2.0, 1);
// }

// TEST_F(MachineSideHardwareMockTest, AdcValueHasCorrectParameters)
// {
//     const int bits = 16;
//     const real_t full_scale = 10.0;
//     meas_mock.configure_voltage_adc(1.0, frequency_t{499.0}, bits, full_scale);
//     meas_mock.update(5.0, -5.0);

//     auto v_ab_val = meas_mock.get_v_ab_counts();
//     auto v_bc_val = meas_mock.get_v_bc_counts();

//     // Verify that the returned adc_value_t objects have the correct conversion parameters
//     // and that the real value is calculated correctly.
//     EXPECT_NEAR(v_ab_val.to_real(), 5.0, 0.01);
//     EXPECT_NEAR(v_bc_val.to_real(), -5.0, 0.01);
// }