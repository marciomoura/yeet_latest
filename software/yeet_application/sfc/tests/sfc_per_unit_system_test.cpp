#include "sfc/sfc_per_unit_system.hpp"

#include <gtest/gtest.h>

#include "yeet_control/units.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

// Test fixture for the sfc_per_unit_system class
class SfcPerUnitSystemTest : public ::testing::Test {
protected:
    // Frequency and electrical parameters with proper units
    static constexpr frequency_t kNominalGridFrequency = frequency_t(50.0);     // 50 Hz grid frequency
    static constexpr frequency_t kNominalMachineFrequency = frequency_t(50.0);  // 50 Hz machine frequency
    static constexpr voltage_t kNominalGridVoltageRMS = voltage_t(2090.0);      // 2090V RMS line voltage
    static constexpr voltage_t kNominalMachineVoltageRMS = voltage_t(1980.0);   // 1980V RMS line voltage

    // Firing angles (dimensionless angles in degrees)
    static constexpr angle_t kNominalGridFiringAngleDeg = angle_t(13.0);  // Nominal firing angle for grid side
    static constexpr angle_t kNominalMachineFiringAngleDeg =
        angle_t(180.0 - 26.84);  // Nominal firing angle for motor side

    // Circuit parameters with proper units
    static constexpr resistance_t kGridSideResistance = resistance_t(0.1552);     // 0.1552 Ohm grid side resistance
    static constexpr inductance_t kGridSideInductance = inductance_t(0.4941e-3);  // 0.4941 mH grid side inductance
    static constexpr resistance_t kMachineSideResistance = resistance_t(0.0788);  // 0.0788 Ohm machine side resistance
    static constexpr inductance_t kMachineSideInductance =
        inductance_t(0.2508e-3);  // 0.2508 mH machine side inductance

    // Power and current ratings with proper units
    static constexpr current_t kBaseDCLinkCurrent = current_t(1044.0);  // 1044 A base DC link current
    static constexpr power_t kMachineRatedPowerVA = power_t(2950e3);    // 2.95 MVA machine rated power
    static constexpr int kMachineNumPoles = 2;                          // Number of poles (dimensionless)

    // Default Mechanical Parameters with proper units
    static constexpr inertia_constant_t kDefaultInertiaConstantH = inertia_constant_t(2.5);  // 2.5 s inertia constant
    static constexpr damping_coefficient_t kDefaultDampingFactorD =
        damping_coefficient_t(0.01);  // 0.01 pu damping factor

    // Use default values from the plant model simulator for consistency
    sfc_per_unit_system pu_system{kNominalGridVoltageRMS, kNominalGridFrequency,     kBaseDCLinkCurrent,
                                  kMachineRatedPowerVA,   kNominalMachineVoltageRMS, kNominalMachineFrequency,
                                  kMachineNumPoles};
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
    EXPECT_EQ(grid_bases.base_frequency.value(), kNominalGridFrequency.value());
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
    EXPECT_EQ(dc_link_bases.base_current.value(), kBaseDCLinkCurrent.value());
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

    EXPECT_EQ(machine_bases.base_power.value(), kMachineRatedPowerVA.value());
    EXPECT_NEAR(machine_bases.base_voltage_peak_phase.value(), expected_base_voltage_peak_phase, 1e-2);
    EXPECT_NEAR(machine_bases.base_current_peak_phase.value(), expected_base_current_peak_phase, 1e-2);
    EXPECT_NEAR(machine_bases.base_impedance.value(), expected_base_impedance, 1e-2);
    EXPECT_EQ(machine_bases.base_frequency.value(), kNominalMachineFrequency.value());
    EXPECT_NEAR(machine_bases.base_inductance.value() * 1000.0, expected_base_inductance_mh, 1e-2);
    EXPECT_NEAR(machine_bases.base_torque.value() / 1000.0, expected_base_torque_knm, 1e-2);
    EXPECT_NEAR(machine_bases.base_flux_linkage.value(), expected_base_flux_linkage, 1e-3);
}