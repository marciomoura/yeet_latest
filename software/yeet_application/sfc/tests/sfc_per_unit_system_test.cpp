#include "sfc/sfc_per_unit_system.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet::control;

// Test fixture for the sfc_per_unit_system class
class SfcPerUnitSystemTest : public ::testing::Test {
protected:
    // These input values are taken directly from the 'steady_state_analysis.py' script
    // to ensure consistency between the model and the implementation.
    static constexpr double transformer_secondary_voltage_ll_rms_v = 2090.0;
    static constexpr double line_frequency_hz = 50.0;
    static constexpr double dc_link_current_rated_a = 1044.0;
    static constexpr double machine_rated_power_va = 2950e3;
    static constexpr double machine_service_voltage_ll_rms_v = 1980.0;
    static constexpr double machine_base_frequency_hz = 50.0;
    static constexpr int machine_number_of_poles = 2;

    sfc_per_unit_system pu_system{transformer_secondary_voltage_ll_rms_v,
                                  line_frequency_hz,
                                  dc_link_current_rated_a,
                                  machine_rated_power_va,
                                  machine_service_voltage_ll_rms_v,
                                  machine_base_frequency_hz,
                                  machine_number_of_poles};
};

TEST_F(SfcPerUnitSystemTest, EvaluateGridSideBaseValue) {
    const auto& grid_bases = pu_system.get_grid_bases();

    // Expected values from the summary table
    constexpr double expected_base_voltage_peak_phase = 1706.48;
    constexpr double expected_base_current_peak_phase = 1151.17;
    constexpr double expected_base_impedance = 1.48;
    constexpr double expected_base_inductance_mh = 4.72;

    EXPECT_NEAR(grid_bases.base_voltage_peak_phase_v, expected_base_voltage_peak_phase, 1e-2);
    EXPECT_NEAR(grid_bases.base_current_peak_phase_a, expected_base_current_peak_phase, 1e-2);
    EXPECT_NEAR(grid_bases.base_impedance_ohm, expected_base_impedance, 1e-2);
    EXPECT_NEAR(grid_bases.base_inductance_h * 1000.0, expected_base_inductance_mh, 1e-2);
    EXPECT_EQ(grid_bases.base_frequency_hz, line_frequency_hz);
}

TEST_F(SfcPerUnitSystemTest, EvaluateDcLinkBaseValue) {
    const auto& dc_link_bases = pu_system.get_dc_link_bases();

    // Expected values from the summary table
    constexpr double expected_base_voltage = 2822.49;
    constexpr double expected_base_impedance = 2.70;
    constexpr double expected_base_inductance_mh = 8.61;

    EXPECT_NEAR(dc_link_bases.base_voltage_v, expected_base_voltage, 1e-2);
    EXPECT_EQ(dc_link_bases.base_current_a, dc_link_current_rated_a);
    EXPECT_NEAR(dc_link_bases.base_impedance_ohm, expected_base_impedance, 1e-2);
    EXPECT_NEAR(dc_link_bases.base_inductance_h * 1000.0, expected_base_inductance_mh, 1e-2);
}

TEST_F(SfcPerUnitSystemTest, EvaluateMachineSideBaseValue) {
    const auto& machine_bases = pu_system.get_machine_bases();

    // Expected values from the summary table
    constexpr double expected_base_voltage_peak_phase = 1616.66;
    constexpr double expected_base_current_peak_phase = 1216.50;
    constexpr double expected_base_impedance = 1.33;
    constexpr double expected_base_inductance_mh = 4.23;
    constexpr double expected_base_torque_knm = 9.39;
    constexpr double expected_base_flux_linkage = 5.146;

    EXPECT_EQ(machine_bases.base_power_va, machine_rated_power_va);
    EXPECT_NEAR(machine_bases.base_voltage_peak_phase_v, expected_base_voltage_peak_phase, 1e-2);
    EXPECT_NEAR(machine_bases.base_current_peak_phase_a, expected_base_current_peak_phase, 1e-2);
    EXPECT_NEAR(machine_bases.base_impedance_ohm, expected_base_impedance, 1e-2);
    EXPECT_EQ(machine_bases.base_frequency_hz, machine_base_frequency_hz);
    EXPECT_NEAR(machine_bases.base_inductance_h * 1000.0, expected_base_inductance_mh, 1e-2);
    EXPECT_NEAR(machine_bases.base_torque_nm / 1000.0, expected_base_torque_knm, 1e-2);
    EXPECT_NEAR(machine_bases.base_flux_linkage_vs_rad, expected_base_flux_linkage, 1e-3);
}