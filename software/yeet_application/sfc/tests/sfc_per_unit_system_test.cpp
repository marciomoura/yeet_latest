#include "sfc/sfc_per_unit_system.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

class SfcPerUnitSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Standard industrial values for testing
        rated_power_va = 2e6;           // 2 MVA
        rated_voltage_ll_rms = 4160.0;  // 4.16 kV
        rated_frequency_hz = 60.0;      // 60 Hz

        pu_system = std::make_unique<sfc_per_unit_system>(rated_power_va, rated_voltage_ll_rms, rated_frequency_hz);
    }

    double rated_power_va;
    double rated_voltage_ll_rms;
    double rated_frequency_hz;
    std::unique_ptr<sfc_per_unit_system> pu_system;
};

TEST_F(SfcPerUnitSystemTest, ConstructorAndBaseValues) {
    ASSERT_NE(pu_system, nullptr);
    EXPECT_DOUBLE_EQ(pu_system->get_base_power(), rated_power_va);
    EXPECT_DOUBLE_EQ(pu_system->get_base_voltage(), rated_voltage_ll_rms);
    EXPECT_DOUBLE_EQ(pu_system->get_base_frequency(), rated_frequency_hz);
}

TEST_F(SfcPerUnitSystemTest, AcSideBaseQuantities) {
    const double expected_base_current = rated_power_va / (control::sqrt_3 * rated_voltage_ll_rms);
    const double expected_base_impedance = (rated_voltage_ll_rms * rated_voltage_ll_rms) / rated_power_va;
    const double expected_base_angular_frequency = control::two_pi<double> * rated_frequency_hz;

    EXPECT_DOUBLE_EQ(pu_system->get_base_current(), expected_base_current);
    EXPECT_DOUBLE_EQ(pu_system->get_base_impedance(), expected_base_impedance);
    EXPECT_DOUBLE_EQ(pu_system->get_base_angular_frequency(), expected_base_angular_frequency);
}

TEST_F(SfcPerUnitSystemTest, PeakBaseQuantities) {
    const double expected_base_voltage_peak =
        rated_voltage_ll_rms * control::line_to_phase_rms_factor * control::rms_to_peak_factor;
    const double expected_base_current_peak =
        (rated_power_va / (control::sqrt_3 * rated_voltage_ll_rms)) * control::rms_to_peak_factor;
    const double expected_base_flux_linkage_peak =
        expected_base_voltage_peak / (control::two_pi<double> * rated_frequency_hz);

    EXPECT_DOUBLE_EQ(pu_system->get_base_voltage_peak(), expected_base_voltage_peak);
    EXPECT_DOUBLE_EQ(pu_system->get_base_current_peak(), expected_base_current_peak);
    EXPECT_DOUBLE_EQ(pu_system->get_base_flux_linkage_peak(), expected_base_flux_linkage_peak);
}

TEST_F(SfcPerUnitSystemTest, DcLinkBaseQuantities) {
    const double expected_base_dc_voltage = sfc_per_unit_system::get_ac_to_dc_voltage_factor() * rated_voltage_ll_rms;
    const double expected_base_dc_current = rated_power_va / expected_base_dc_voltage;
    const double expected_base_dc_impedance = expected_base_dc_voltage / expected_base_dc_current;

    EXPECT_DOUBLE_EQ(pu_system->get_base_dc_voltage(), expected_base_dc_voltage);
    EXPECT_DOUBLE_EQ(pu_system->get_base_dc_current(), expected_base_dc_current);
    EXPECT_DOUBLE_EQ(pu_system->get_base_dc_impedance(), expected_base_dc_impedance);
}

TEST_F(SfcPerUnitSystemTest, ConversionFactors) {
    constexpr double expected_ac_to_dc_voltage_factor = 3.0 * control::sqrt_2 / yeet::control::pi<double>;
    constexpr double expected_dc_to_ac_current_factor = yeet::control::pi<double> / (3.0 * control::sqrt_2);

    EXPECT_DOUBLE_EQ(sfc_per_unit_system::get_ac_to_dc_voltage_factor(), expected_ac_to_dc_voltage_factor);
    EXPECT_DOUBLE_EQ(sfc_per_unit_system::get_dc_to_ac_current_factor(), expected_dc_to_ac_current_factor);
}

}  // namespace yeet::application::sfc
