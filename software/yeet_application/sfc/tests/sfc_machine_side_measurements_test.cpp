#include "sfc/sfc_machine_side_measurements.hpp"

#include <gtest/gtest.h>

#include "sfc/sfc_per_unit_system.hpp"
#include "yeet_control/precomputed_divisor.hpp"

using namespace yeet;
using namespace yeet::application::sfc;
using namespace yeet::control;

class SfcMachineSideMeasurementsTest : public ::testing::Test {
protected:
    sfc_per_unit_system::machine_side_bases bases;

    void SetUp() override
    {
        bases.base_voltage_peak_phase = precomputed_divisor<voltage_t>{voltage_t{100.0}};  // 100V peak phase
        bases.base_current_peak_phase = precomputed_divisor<current_t>{current_t{10.0}};   // 10A peak phase
    }
};

// --- Tests for sfc_machine_side_measurements_ideal ---
TEST_F(SfcMachineSideMeasurementsTest, IdealUpdateConvertsToPuCorrectly)
{
    sfc_machine_side_measurements_ideal meas;

    const abc<voltage_t> raw_voltages{voltage_t{50.0}, voltage_t{-25.0}, voltage_t{-25.0}};
    // Currents are not measured on machine side, but the ideal class takes them for consistency
    const abc<current_t> raw_currents{current_t{0.0}, current_t{0.0}, current_t{0.0}};

    meas.update(bases, raw_voltages, raw_currents);

    const auto& voltage_pu = meas.get_voltage_pu();
    EXPECT_NEAR(voltage_pu.a().value(), 0.5, 1e-9);
    EXPECT_NEAR(voltage_pu.b().value(), -0.25, 1e-9);
    EXPECT_NEAR(voltage_pu.c().value(), -0.25, 1e-9);

    const auto& current_pu = meas.get_current_pu();
    EXPECT_NEAR(current_pu.a().value(), 0.0, 1e-9);
    EXPECT_NEAR(current_pu.b().value(), 0.0, 1e-9);
    EXPECT_NEAR(current_pu.c().value(), 0.0, 1e-9);
}

// --- Tests for sfc_machine_side_measurements (Realistic) ---
class SfcMachineSideMeasurementsRealisticTest : public SfcMachineSideMeasurementsTest {
protected:
    sfc_machine_side_measurements meas;
};

TEST_F(SfcMachineSideMeasurementsRealisticTest, InitialStateIsZero)
{
    EXPECT_EQ(meas.get_raw_v_ab(), 0.0);
    EXPECT_EQ(meas.get_raw_v_bc(), 0.0);

    EXPECT_EQ(meas.get_voltage_pu().a().value(), 0.0);
    EXPECT_EQ(meas.get_voltage_pu().b().value(), 0.0);
    EXPECT_EQ(meas.get_voltage_pu().c().value(), 0.0);

    EXPECT_EQ(meas.get_current_pu().a().value(), 0.0);
}

TEST_F(SfcMachineSideMeasurementsRealisticTest, ResetWorks)
{
    meas.configure_voltage_conversion(12, 5.0);

    const adc_value_t v_ab(12, 5.0, 100);
    meas.update(bases, v_ab, {});

    ASSERT_NE(meas.get_raw_v_ab(), 0.0);

    meas.reset();

    EXPECT_EQ(meas.get_raw_v_ab(), 0.0);
    EXPECT_EQ(meas.get_raw_v_bc(), 0.0);
    EXPECT_EQ(meas.get_voltage_pu().a().value(), 0.0);
}

TEST_F(SfcMachineSideMeasurementsRealisticTest, UpdateFromAdcConvertsCorrectly)
{
    const int voltage_bits = 12;
    const real_t voltage_full_scale = 5.0;

    meas.configure_voltage_conversion(voltage_bits, voltage_full_scale);

    // Create ADC values representing half of the positive full-scale range
    const std::int32_t v_counts = (1 << (voltage_bits - 1)) / 2;  // 1023 for 12-bit

    const adc_value_t v_ab(voltage_bits, voltage_full_scale, v_counts);
    const adc_value_t v_bc(voltage_bits, voltage_full_scale, -v_counts);

    meas.update(bases, v_ab, v_bc);

    // 1. Check raw value conversion
    EXPECT_NEAR(meas.get_raw_v_ab(), voltage_full_scale / 2.0, 0.01);
    EXPECT_NEAR(meas.get_raw_v_bc(), -voltage_full_scale / 2.0, 0.01);

    // 2. Check third-phase calculation (assuming balanced system)
    // v_ca = -(v_ab + v_bc) -> should be near zero
    const auto& voltage_pu = meas.get_voltage_pu();
    EXPECT_NEAR(voltage_pu.c().value(), 0.0, 0.01);  // v_ca should be ~0

    // 3. Check final per-unit conversion
    const real_t expected_v_pu = (voltage_full_scale / 2.0) / bases.base_voltage_peak_phase.value();
    EXPECT_NEAR(voltage_pu.a().value(), expected_v_pu, 0.01);
    EXPECT_NEAR(voltage_pu.b().value(), -expected_v_pu, 0.01);

    // 4. Check that currents are zero
    const auto& current_pu = meas.get_current_pu();
    EXPECT_EQ(current_pu.a().value(), 0.0);
    EXPECT_EQ(current_pu.b().value(), 0.0);
    EXPECT_EQ(current_pu.c().value(), 0.0);
}
