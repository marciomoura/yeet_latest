#include "sfc/sfc_commutation_mask.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet;

class SfcCommutationMaskTest : public ::testing::Test {
protected:
    sfc_commutation_mask mask;

    // Test parameters
    const frequency_t test_frequency = frequency_t(60.0);
    const real_t dt = 1.0 / 20000.0;  // 20 kHz sample rate
    const voltage_pu_t test_magnitude{1.0F};
};

// Test that when not commuting, the mask passes the measured voltage through unchanged.
TEST_F(SfcCommutationMaskTest, PassThroughMode) {
    const abc<voltage_pu_t> measured_voltages{voltage_pu_t{1.0F}, voltage_pu_t{-0.5F}, voltage_pu_t{-0.5F}};
    const angle<zero_to_two_pi> pll_angle{0.0F};
    const bool is_commuting = false;

    const auto output_voltages = mask.update(measured_voltages, pll_angle, test_frequency, is_commuting, dt);

    EXPECT_EQ(output_voltages.a(), measured_voltages.a());
    EXPECT_EQ(output_voltages.b(), measured_voltages.b());
    EXPECT_EQ(output_voltages.c(), measured_voltages.c());
}

// Test the transition from masking back to pass-through mode.
TEST_F(SfcCommutationMaskTest, MaskingEnds) {
    angle<zero_to_two_pi> pll_angle{0.0F};

    // --- Step 1: Establish a magnitude ---
    const auto pre_notch_voltages = dq<voltage_pu_t>{test_magnitude, voltage_pu_t{0.0F}}.to_abc(pll_angle);
    mask.update(pre_notch_voltages, pll_angle, test_frequency, false, dt);

    // --- Step 2: Enter masking mode ---
    pll_angle += angle<zero_to_two_pi>(test_frequency.value() * two_pi * dt);
    const abc<voltage_pu_t> notched_voltages{voltage_pu_t{0.0F}, voltage_pu_t{0.0F}, voltage_pu_t{0.0F}};
    mask.update(notched_voltages, pll_angle, test_frequency, true, dt);

    // --- Step 3: Commutation ends. The mask should revert to pass-through. ---
    pll_angle += angle<zero_to_two_pi>(test_frequency.value() * two_pi * dt);
    const auto post_notch_voltages = dq<voltage_pu_t>{test_magnitude, voltage_pu_t{0.0F}}.to_abc(pll_angle);
    auto output = mask.update(post_notch_voltages, pll_angle, test_frequency, false, dt);

    EXPECT_EQ(output.a(), post_notch_voltages.a());
    EXPECT_EQ(output.b(), post_notch_voltages.b());
    EXPECT_EQ(output.c(), post_notch_voltages.c());
}