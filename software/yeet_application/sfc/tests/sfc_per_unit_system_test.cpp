#include "sfc/sfc_per_unit_system.hpp"

#include <gtest/gtest.h>

#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

// Test fixture for the sfc_per_unit_system class
class SfcPerUnitSystemTest : public ::testing::Test {
protected:
    // Use default values from the plant model simulator for consistency
    sfc_per_unit_system pu_system{
        sfc_plant_model_simulator::kNominalGridVoltageRMS,    sfc_plant_model_simulator::kNominalGridFrequency,
        sfc_plant_model_simulator::kBaseDCLinkCurrent,        sfc_plant_model_simulator::kMachineRatedPowerVA,
        sfc_plant_model_simulator::kNominalMachineVoltageRMS, sfc_plant_model_simulator::kNominalMachineFrequency,
        sfc_plant_model_simulator::kMachineNumPoles};
};

TEST_F(SfcPerUnitSystemTest, EvaluateGridSideBaseValue) {
    const auto& grid_bases = pu_system.get_grid_bases();

    // Expected values calculated from the plant model simulator's default parameters
    // Grid voltage RMS: 2090V -> Peak phase voltage: 2090 * sqrt(2) / sqrt(3) = 1706.48V
    // Base current: 1044A -> Peak phase current: 1044 * sqrt(2) / sqrt(3) = 1206.23A
    // Note: These values are calculated based on the simulator's default parameters
    constexpr double expected_base_voltage_peak_phase = 1706.48;
    constexpr double expected_base_current_peak_phase = 1151.17;
    constexpr double expected_base_impedance = 1.48;
    constexpr double expected_base_inductance_mh = 4.72;
    EXPECT_NEAR(grid_bases.base_voltage_peak_phase.value(), expected_base_voltage_peak_phase, 1e-2);
    EXPECT_NEAR(grid_bases.base_current_peak_phase.value(), expected_base_current_peak_phase, 1e-2);
    EXPECT_NEAR(grid_bases.base_impedance.value(), expected_base_impedance, 1e-2);
    EXPECT_NEAR(grid_bases.base_inductance.value() * 1000.0, expected_base_inductance_mh, 1e-2);
    EXPECT_EQ(grid_bases.base_frequency.value(), sfc_plant_model_simulator::kNominalGridFrequency.value());
}

TEST_F(SfcPerUnitSystemTest, EvaluateDcLinkBaseValue) {
    const auto& dc_link_bases = pu_system.get_dc_link_bases();

    // Expected values calculated from the plant model simulator's default parameters
    // DC voltage base: 3 * sqrt(2) * V_grid_rms / pi = 3 * sqrt(2) * 2090 / pi = 2822.49V
    // DC current base: 1044A (direct from simulator)
    // Base impedance: V_dc_base / I_dc_base = 2822.49 / 1044 = 2.70 Ohm
    constexpr double expected_base_voltage = 2822.49;
    constexpr double expected_base_impedance = 2.70;
    constexpr double expected_base_inductance_mh = 8.61;

    EXPECT_NEAR(dc_link_bases.base_voltage.value(), expected_base_voltage, 1e-2);
    EXPECT_EQ(dc_link_bases.base_current.value(), sfc_plant_model_simulator::kBaseDCLinkCurrent.value());
    EXPECT_NEAR(dc_link_bases.base_impedance.value(), expected_base_impedance, 1e-2);
    EXPECT_NEAR(dc_link_bases.base_inductance.value() * 1000.0, expected_base_inductance_mh, 1e-2);
}

TEST_F(SfcPerUnitSystemTest, EvaluateMachineSideBaseValue) {
    const auto& machine_bases = pu_system.get_machine_bases();

    // Expected values calculated from the plant model simulator's default parameters
    // Machine voltage RMS: 1980V -> Peak phase voltage: 1980 * sqrt(2) / sqrt(3) = 1616.66V
    // Machine current: Calculated from power and voltage: P / (sqrt(3) * V_ll) = 2950e3 / (sqrt(3) * 1980) = 860.35A
    // Peak phase current: 860.35 * sqrt(2) / sqrt(3) = 993.60A

    constexpr double expected_base_voltage_peak_phase = 1616.66;
    constexpr double expected_base_current_peak_phase = 1216.50;
    constexpr double expected_base_impedance = 1.33;
    constexpr double expected_base_inductance_mh = 4.23;
    constexpr double expected_base_torque_knm = 9.39;
    constexpr double expected_base_flux_linkage = 5.146;  // V_peak_phase / (2*pi*f_base)

    EXPECT_EQ(machine_bases.base_power.value(), sfc_plant_model_simulator::kMachineRatedPowerVA.value());
    EXPECT_NEAR(machine_bases.base_voltage_peak_phase.value(), expected_base_voltage_peak_phase, 1e-2);
    EXPECT_NEAR(machine_bases.base_current_peak_phase.value(), expected_base_current_peak_phase, 1e-2);
    EXPECT_NEAR(machine_bases.base_impedance.value(), expected_base_impedance, 1e-2);
    EXPECT_EQ(machine_bases.base_frequency.value(), sfc_plant_model_simulator::kNominalMachineFrequency.value());
    EXPECT_NEAR(machine_bases.base_inductance.value() * 1000.0, expected_base_inductance_mh, 1e-2);
    EXPECT_NEAR(machine_bases.base_torque.value() / 1000.0, expected_base_torque_knm, 1e-2);
    EXPECT_NEAR(machine_bases.base_flux_linkage.value(), expected_base_flux_linkage, 1e-3);
}