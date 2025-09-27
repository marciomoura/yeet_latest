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
    const abc<current_t> raw_currents{current_t{10.0}, current_t{5.0}, current_t{-10.0}};

    meas.update(bases, raw_voltages, raw_currents);

    const auto& voltage_pu = meas.get_voltage_pu();
    EXPECT_NEAR(voltage_pu.a().value(), 0.5, 1e-9);
    EXPECT_NEAR(voltage_pu.b().value(), -0.25, 1e-9);
    EXPECT_NEAR(voltage_pu.c().value(), -0.25, 1e-9);

    const auto& current_pu = meas.get_current_pu();
    EXPECT_NEAR(current_pu.a().value(), 1.0, 1e-9);
    EXPECT_NEAR(current_pu.b().value(), 0.5, 1e-9);
    EXPECT_NEAR(current_pu.c().value(), -1.0, 1e-9);
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

    adc_converter v_conv(12, 5.0);
    const auto v_ab = v_conv.to_counts(5.0 * 100 / 2047.0);
    meas.update(bases, v_ab, {});

    ASSERT_NE(meas.get_raw_v_ab(), 0.0);

    meas.reset();

    EXPECT_EQ(meas.get_raw_v_ab(), 0.0);
    EXPECT_EQ(meas.get_raw_v_bc(), 0.0);
    EXPECT_EQ(meas.get_voltage_pu().a().value(), 0.0);
}

TEST_F(SfcMachineSideMeasurementsRealisticTest, UpdateFromAdcConvertsCorrectly)
{
    // configure adc conversion (card input full-scale volts)
    const int voltage_bits = 16;
    const real_t adc_full_scale_voltage = 5.0;
    meas.configure_voltage_conversion(voltage_bits, adc_full_scale_voltage);

    // configure potential transformer (primary line-line -> secondary line-line)
    const voltage_t primary_line_line_voltage{3000.0};
    const voltage_t secondary_line_line_voltage{100.0};
    meas.configure_voltage_transformer(primary_line_line_voltage.value(), secondary_line_line_voltage.value());
    const real_t potential_transformer_ratio =
        primary_line_line_voltage.value() / secondary_line_line_voltage.value();  // This is 30.0

    // configure transducer gain (secondary -> card). in implementation: voltage_secondary = voltage_card / gain
    const real_t voltage_transducer_gain = 0.01;  // card voltage = secondary * gain
    meas.configure_voltage_transducer_gain(voltage_transducer_gain);

    // define desired primary phase (a,b,c) voltages (peak, phase-neutral) summing to zero
    const voltage_t voltage_a_primary{40.0};   // 0.4 pu with 100 V base
    const voltage_t voltage_b_primary{-10.0};  // -0.1 pu with 100 V base
    const voltage_t voltage_c_primary{-30.0};  // -0.3 pu with 100 V base
    ASSERT_NEAR(voltage_a_primary.value() + voltage_b_primary.value() + voltage_c_primary.value(), 0.0, 1e-9);

    // The implementation scales reconstructed *phase* voltages by the potential_transformer_ratio.
    // We must work backwards from the desired primary phases to find the secondary line voltages.

    // 1. Calculate the secondary *phase* voltages that would result in the desired primary phases.
    const voltage_t voltage_a_secondary = voltage_a_primary / potential_transformer_ratio;
    const voltage_t voltage_b_secondary = voltage_b_primary / potential_transformer_ratio;
    const voltage_t voltage_c_secondary = voltage_c_primary / potential_transformer_ratio;

    // 2. From these secondary phase voltages, calculate the secondary *line-line* voltages.
    // These are the values the hardware would actually measure on the secondary side.
    const voltage_t voltage_ab_secondary = voltage_a_secondary - voltage_b_secondary;
    const voltage_t voltage_bc_secondary = voltage_b_secondary - voltage_c_secondary;

    // 3. Apply transducer gain to obtain card voltage (what adc sees)
    const voltage_t voltage_ab_card = voltage_ab_secondary * voltage_transducer_gain;
    const voltage_t voltage_bc_card = voltage_bc_secondary * voltage_transducer_gain;

    // 4. Convert to adc counts
    adc_converter adc_converter_instance(voltage_bits, adc_full_scale_voltage);
    const auto voltage_ab_counts = adc_converter_instance.to_counts(voltage_ab_card.value());
    const auto voltage_bc_counts = adc_converter_instance.to_counts(voltage_bc_card.value());

    // 5. Run update with counts
    meas.update(bases, voltage_ab_counts, voltage_bc_counts);

    // 6. Verify reconstructed per-unit phase voltages
    const auto& voltage_per_unit = meas.get_voltage_pu();
    const voltage_t voltage_base_phase_peak = bases.base_voltage_peak_phase.value();
    EXPECT_NEAR(voltage_per_unit.a().value(), voltage_a_primary.value() / voltage_base_phase_peak.value(), 1e-3);
    EXPECT_NEAR(voltage_per_unit.b().value(), voltage_b_primary.value() / voltage_base_phase_peak.value(), 1e-3);
    EXPECT_NEAR(voltage_per_unit.c().value(), voltage_c_primary.value() / voltage_base_phase_peak.value(), 1e-3);

    // 7. Zero-sequence consistency
    EXPECT_NEAR(voltage_per_unit.a().value() + voltage_per_unit.b().value() + voltage_per_unit.c().value(), 0.0, 1e-6);

    // 8. Reconstruct line-line from recovered phases and compare
    const voltage_t voltage_ab_recovered =
        (voltage_per_unit.a().value() - voltage_per_unit.b().value()) * voltage_base_phase_peak;
    const voltage_t voltage_bc_recovered =
        (voltage_per_unit.b().value() - voltage_per_unit.c().value()) * voltage_base_phase_peak;
    const voltage_t voltage_ab_primary = voltage_a_primary - voltage_b_primary;
    const voltage_t voltage_bc_primary = voltage_b_primary - voltage_c_primary;
    EXPECT_NEAR(voltage_ab_recovered.value(), voltage_ab_primary.value(), 0.2);
    EXPECT_NEAR(voltage_bc_recovered.value(), voltage_bc_primary.value(), 0.2);
}
