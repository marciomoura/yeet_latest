#include "sfc/sfc_grid_side_measurements.hpp"

#include <gtest/gtest.h>

#include "sfc/sfc_per_unit_system.hpp"
#include "yeet_control/precomputed_divisor.hpp"

using namespace yeet;
using namespace yeet::application::sfc;
using namespace yeet::control;

class SfcGridSideMeasurementsTest : public ::testing::Test {
protected:
    sfc_per_unit_system::grid_side_bases bases;

    void SetUp() override
    {
        bases.base_voltage_peak_phase = precomputed_divisor<voltage_t>{voltage_t{100.0}};  // 100V peak phase
        bases.base_current_peak_phase = precomputed_divisor<current_t>{current_t{10.0}};   // 10A peak phase
    }
};

// --- Tests for sfc_grid_side_measurements_ideal ---
TEST_F(SfcGridSideMeasurementsTest, IdealUpdateConvertsToPuCorrectly)
{
    sfc_grid_side_measurements_ideal measurements;

    const abc<voltage_t> raw_voltages{voltage_t{50.0}, voltage_t{-25.0}, voltage_t{-25.0}};
    const abc<current_t> raw_currents{current_t{5.0}, current_t{-2.5}, current_t{-2.5}};

    measurements.update(bases, raw_voltages, raw_currents);

    const auto& voltage_per_unit = measurements.get_voltage_pu();
    EXPECT_NEAR(voltage_per_unit.a().value(), 0.5, 1e-9);
    EXPECT_NEAR(voltage_per_unit.b().value(), -0.25, 1e-9);
    EXPECT_NEAR(voltage_per_unit.c().value(), -0.25, 1e-9);

    const auto& current_per_unit = measurements.get_current_pu();
    EXPECT_NEAR(current_per_unit.a().value(), 0.5, 1e-9);
    EXPECT_NEAR(current_per_unit.b().value(), -0.25, 1e-9);
    EXPECT_NEAR(current_per_unit.c().value(), -0.25, 1e-9);
}

// --- Tests for sfc_grid_side_measurements (Realistic) ---
class SfcGridSideMeasurementsRealisticTest : public ::testing::Test {
protected:
    sfc_grid_side_measurements measurements;
    sfc_per_unit_system::grid_side_bases bases;

    void SetUp() override
    {
        bases.base_voltage_peak_phase = precomputed_divisor<voltage_t>{voltage_t{1000.0}};  // 1kV peak phase
        bases.base_current_peak_phase = precomputed_divisor<current_t>{current_t{100.0}};   // 100A peak phase
    }
};

TEST_F(SfcGridSideMeasurementsRealisticTest, InitialStateIsZero)
{
    EXPECT_EQ(measurements.get_raw_v_ab(), 0.0);
    EXPECT_EQ(measurements.get_raw_v_bc(), 0.0);
    EXPECT_EQ(measurements.get_raw_i_a(), 0.0);
    EXPECT_EQ(measurements.get_raw_i_c(), 0.0);

    EXPECT_EQ(measurements.get_voltage_pu().a().value(), 0.0);
    EXPECT_EQ(measurements.get_current_pu().a().value(), 0.0);
}

TEST_F(SfcGridSideMeasurementsRealisticTest, ResetWorks)
{
    measurements.configure_voltage_conversion(12, 5.0);
    measurements.configure_current_conversion(16, 10.0);

    adc_converter voltage_converter(12, 5.0);
    adc_converter current_converter(16, 10.0);
    const auto voltage_ab_test_counts = voltage_converter.to_counts(5.0 * 100 / 2047.0);  // arbitrary small non-zero
    const auto current_a_test_counts = current_converter.to_counts(10.0 * 200 / 32767.0);
    measurements.update(bases, voltage_ab_test_counts, {}, current_a_test_counts, {});

    ASSERT_NE(measurements.get_raw_v_ab(), 0.0);
    ASSERT_NE(measurements.get_raw_i_a(), 0.0);

    measurements.reset();

    EXPECT_EQ(measurements.get_raw_v_ab(), 0.0);
    EXPECT_EQ(measurements.get_raw_i_a(), 0.0);
    EXPECT_EQ(measurements.get_voltage_pu().a().value(), 0.0);
    EXPECT_EQ(measurements.get_current_pu().a().value(), 0.0);
}

TEST_F(SfcGridSideMeasurementsRealisticTest, UpdateFromAdcConvertsCorrectly)
{
    // 1. Configure the entire measurement chain
    const int voltage_bits = 16;
    const real_t adc_full_scale_voltage = 5.0;
    measurements.configure_voltage_conversion(voltage_bits, adc_full_scale_voltage);
    measurements.configure_current_conversion(voltage_bits, adc_full_scale_voltage);
    measurements.configure_voltage_transformer(13800.0, 115.0);  // 13.8kV / 115V PT
    measurements.configure_current_transformer(500.0, 5.0);      // 500A / 5A CT
    measurements.configure_voltage_transducer_gain(0.04);        // Scales 115V secondary range to fit ADC
    measurements.configure_current_transducer_gain(0.8);         // Scales 5A secondary range to fit ADC

    const voltage_t potential_transformer_ratio{13800.0 / 115.0};
    const current_t current_transformer_ratio{500.0 / 5.0};
    const real_t voltage_transducer_gain = 0.04;
    const real_t current_transducer_gain = 0.8;

    // 2. Define desired PRIMARY physical values
    // Voltages (sum=0)
    const voltage_t voltage_a_primary{800.0};   // 0.8 pu
    const voltage_t voltage_b_primary{-200.0};  // -0.2 pu
    const voltage_t voltage_c_primary{-600.0};  // -0.6 pu
    ASSERT_NEAR(voltage_a_primary.value() + voltage_b_primary.value() + voltage_c_primary.value(), 0.0, 1e-9);
    // Currents (sum=0)
    const current_t current_a_primary{50.0};   // 0.5 pu
    const current_t current_b_primary{20.0};   // 0.2 pu
    const current_t current_c_primary{-70.0};  // -0.7 pu
    ASSERT_NEAR(current_a_primary.value() + current_b_primary.value() + current_c_primary.value(), 0.0, 1e-9);

    // 3. Work backwards to calculate expected ADC counts
    // Voltages
    const voltage_t voltage_ab_primary = voltage_a_primary - voltage_b_primary;
    const voltage_t voltage_bc_primary = voltage_b_primary - voltage_c_primary;
    const voltage_t voltage_ab_secondary = voltage_ab_primary / potential_transformer_ratio.value();
    const voltage_t voltage_bc_secondary = voltage_bc_primary / potential_transformer_ratio.value();
    const voltage_t voltage_ab_card = voltage_ab_secondary * voltage_transducer_gain;
    const voltage_t voltage_bc_card = voltage_bc_secondary * voltage_transducer_gain;
    // Currents
    const current_t current_a_secondary = current_a_primary / current_transformer_ratio.value();
    const current_t current_c_secondary = current_c_primary / current_transformer_ratio.value();
    const voltage_t current_a_card_voltage{current_a_secondary.value() * current_transducer_gain};
    const voltage_t current_c_card_voltage{current_c_secondary.value() * current_transducer_gain};

    // Convert to counts
    adc_converter adc_converter_instance(voltage_bits, adc_full_scale_voltage);
    const auto voltage_ab_counts = adc_converter_instance.to_counts(voltage_ab_card.value());
    const auto voltage_bc_counts = adc_converter_instance.to_counts(voltage_bc_card.value());
    const auto current_a_counts = adc_converter_instance.to_counts(current_a_card_voltage.value());
    const auto current_c_counts = adc_converter_instance.to_counts(current_c_card_voltage.value());

    // 4. Run update with calculated counts
    measurements.update(bases, voltage_ab_counts, voltage_bc_counts, current_a_counts, current_c_counts);

    // 5. Verify final per-unit values
    const auto& voltage_per_unit = measurements.get_voltage_pu();
    const auto& current_per_unit = measurements.get_current_pu();
    const voltage_t voltage_base = bases.base_voltage_peak_phase.value();
    const current_t current_base = bases.base_current_peak_phase.value();

    EXPECT_NEAR(voltage_per_unit.a().value(), voltage_a_primary.value() / voltage_base.value(), 1e-3);
    EXPECT_NEAR(voltage_per_unit.b().value(), voltage_b_primary.value() / voltage_base.value(), 1e-3);
    EXPECT_NEAR(voltage_per_unit.c().value(), voltage_c_primary.value() / voltage_base.value(), 1e-3);

    EXPECT_NEAR(current_per_unit.a().value(), current_a_primary.value() / current_base.value(), 1e-4);
    EXPECT_NEAR(current_per_unit.b().value(), current_b_primary.value() / current_base.value(), 1e-4);
    EXPECT_NEAR(current_per_unit.c().value(), current_c_primary.value() / current_base.value(), 1e-4);
}