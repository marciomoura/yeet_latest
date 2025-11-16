#include "sfc/sfc_vhz_protection.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class VhzProtectionTest : public ::testing::Test {
protected:
    static constexpr duration_t sampling_period{0.0001};  // 100µs (10 kHz)

    VhzProtectionTest() : protection(sampling_period) {}

    void SetUp() override { protection.reset(); }

    // Helper: Run N steps with given voltage and frequency
    void run_steps(int steps, voltage_pu_t voltage, frequency_t frequency)
    {
        for (int i = 0; i < steps; ++i) {
            protection.update(voltage, frequency);
        }
    }

    sfc_vhz_protection protection;
};

// ==================== Ratio Calculation Tests ====================

TEST_F(VhzProtectionTest, CalculatesVhzRatioCorrectly)
{
    // Rated conditions: V = 1.0 pu, f = 50 Hz
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});  // 0.02 pu/Hz

    // Test 1: Rated conditions → V/Hz = 1.0 pu
    protection.update(voltage_pu_t{1.0}, frequency_t{50.0});
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), 1.0, 1e-6);

    // Test 2: Double voltage, same frequency → V/Hz = 2.0 pu (over-flux)
    protection.update(voltage_pu_t{2.0}, frequency_t{50.0});
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), 2.0, 1e-6);

    // Test 3: Rated voltage, double frequency → V/Hz = 0.5 pu (under-flux)
    protection.update(voltage_pu_t{1.0}, frequency_t{100.0});
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), 0.5, 1e-6);

    // Test 4: Half voltage, half frequency → V/Hz = 1.0 pu (rated)
    protection.update(voltage_pu_t{0.5}, frequency_t{25.0});
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), 1.0, 1e-6);
}

TEST_F(VhzProtectionTest, HandlesZeroFrequencySafely)
{
    // Zero frequency should not cause division by zero
    protection.update(voltage_pu_t{1.0}, frequency_t{0.0});
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), 0.0, 1e-6);
    EXPECT_FALSE(protection.is_over_flux_tripped());
    EXPECT_FALSE(protection.is_under_flux_tripped());

    // Very low frequency (below 0.1 Hz threshold) should also be safe
    protection.update(voltage_pu_t{1.0}, frequency_t{0.05});
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), 0.0, 1e-6);
}

TEST_F(VhzProtectionTest, ReturnsToNormalCalculationAboveMinimumFrequency)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});

    // Below minimum frequency → forced to zero
    protection.update(voltage_pu_t{1.0}, frequency_t{0.05});
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), 0.0, 1e-6);

    // Above minimum frequency → normal calculation
    protection.update(voltage_pu_t{1.0}, frequency_t{10.0});  // 10 Hz
    real_t expected_vhz = (1.0 / 10.0) / (1.0 / 50.0);        // = 5.0 pu
    EXPECT_NEAR(protection.get_vhz_ratio_pu(), expected_vhz, 1e-6);
}

// ==================== Over-Flux Protection Tests ====================

TEST_F(VhzProtectionTest, OverFluxTripsWhenThresholdExceeded)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});  // 110% trip
    protection.configure_over_flux_trip_delay(duration_t{0.01});     // 10ms = 100 steps at 10 kHz

    // Apply over-flux condition: V/Hz = 1.15 pu (exceeds 1.10 threshold)
    voltage_pu_t voltage{1.15};
    frequency_t frequency{50.0};

    // Before delay expires → no trip
    run_steps(99, voltage, frequency);
    EXPECT_FALSE(protection.is_over_flux_tripped());

    // After delay expires → trip
    run_steps(1, voltage, frequency);
    EXPECT_TRUE(protection.is_over_flux_tripped());
}

TEST_F(VhzProtectionTest, OverFluxDoesNotTripBelowThreshold)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});

    // Apply safe condition: V/Hz = 1.05 pu (below 1.10 threshold)
    run_steps(200, voltage_pu_t{1.05}, frequency_t{50.0});

    EXPECT_FALSE(protection.is_over_flux_tripped());
}

TEST_F(VhzProtectionTest, OverFluxTripLatches)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});

    // Trip the protection
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_over_flux_tripped());

    // Return to safe condition → trip remains latched
    run_steps(100, voltage_pu_t{1.0}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_over_flux_tripped());
}

TEST_F(VhzProtectionTest, OverFluxWarningActivatesAtLowerThreshold)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_warning_threshold(flux_pu_t{1.05});  // 105% warning
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});     // 110% trip
    protection.configure_over_flux_warning_delay(duration_t{0.005});    // 5ms
    protection.configure_over_flux_trip_delay(duration_t{0.01});        // 10ms

    // Apply warning-level condition: V/Hz = 1.07 pu
    run_steps(50, voltage_pu_t{1.07}, frequency_t{50.0});  // 5ms elapsed

    // Warning should be active, trip should not
    EXPECT_TRUE(protection.is_over_flux_warning());
    EXPECT_FALSE(protection.is_over_flux_tripped());
}

TEST_F(VhzProtectionTest, OverFluxWarningClearImmediately)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_warning_threshold(flux_pu_t{1.05});
    protection.configure_over_flux_warning_delay(duration_t{0.005});

    // Activate warning
    run_steps(50, voltage_pu_t{1.07}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_over_flux_warning());

    // Return to safe condition → warning clears immediately (not latched)
    protection.update(voltage_pu_t{1.0}, frequency_t{50.0});
    EXPECT_FALSE(protection.is_over_flux_warning());
}

// ==================== Under-Flux Protection Tests ====================

TEST_F(VhzProtectionTest, UnderFluxTripsWhenBelowThreshold)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80});  // 80% trip
    protection.configure_under_flux_trip_delay(duration_t{0.05});     // 50ms = 500 steps at 10 kHz

    // Apply under-flux condition: V/Hz = 0.70 pu (below 0.80 threshold)
    voltage_pu_t voltage{0.70};
    frequency_t frequency{50.0};

    // Before delay expires → no trip
    run_steps(499, voltage, frequency);
    EXPECT_FALSE(protection.is_under_flux_tripped());

    // After delay expires → trip
    run_steps(1, voltage, frequency);
    EXPECT_TRUE(protection.is_under_flux_tripped());
}

TEST_F(VhzProtectionTest, UnderFluxDoesNotTripAboveThreshold)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80});
    protection.configure_under_flux_trip_delay(duration_t{0.05});

    // Apply safe condition: V/Hz = 0.85 pu (above 0.80 threshold)
    run_steps(1000, voltage_pu_t{0.85}, frequency_t{50.0});

    EXPECT_FALSE(protection.is_under_flux_tripped());
}

TEST_F(VhzProtectionTest, UnderFluxTripLatches)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80});
    protection.configure_under_flux_trip_delay(duration_t{0.05});

    // Trip the protection
    run_steps(500, voltage_pu_t{0.70}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_under_flux_tripped());

    // Return to safe condition → trip remains latched
    run_steps(100, voltage_pu_t{1.0}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_under_flux_tripped());
}

TEST_F(VhzProtectionTest, UnderFluxWarningActivatesAtHigherThreshold)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_under_flux_warning_threshold(flux_pu_t{0.90});  // 90% warning
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80});     // 80% trip
    protection.configure_under_flux_warning_delay(duration_t{0.02});     // 20ms
    protection.configure_under_flux_trip_delay(duration_t{0.05});        // 50ms

    // Apply warning-level condition: V/Hz = 0.85 pu
    run_steps(200, voltage_pu_t{0.85}, frequency_t{50.0});  // 20ms elapsed

    // Warning should be active, trip should not
    EXPECT_TRUE(protection.is_under_flux_warning());
    EXPECT_FALSE(protection.is_under_flux_tripped());
}

// ==================== Dual-Mode Tests ====================

TEST_F(VhzProtectionTest, OverFluxAndUnderFluxAreIndependent)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80});
    protection.configure_under_flux_trip_delay(duration_t{0.05});

    // Trip over-flux
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_over_flux_tripped());
    EXPECT_FALSE(protection.is_under_flux_tripped());

    // Clear fault condition and reset
    run_steps(10, voltage_pu_t{1.0}, frequency_t{50.0});  // Return to safe mid-range
    protection.reset();
    EXPECT_FALSE(protection.is_over_flux_tripped());
    EXPECT_FALSE(protection.is_under_flux_tripped());

    // Now trip under-flux
    run_steps(500, voltage_pu_t{0.70}, frequency_t{50.0});
    EXPECT_FALSE(protection.is_over_flux_tripped());
    EXPECT_TRUE(protection.is_under_flux_tripped());
}

TEST_F(VhzProtectionTest, IsAnyTrippedReturnsTrueForEitherCondition)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80});
    protection.configure_under_flux_trip_delay(duration_t{0.05});

    EXPECT_FALSE(protection.is_any_tripped());

    // Over-flux trip
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_any_tripped());

    // Reset and under-flux trip
    protection.reset();
    run_steps(500, voltage_pu_t{0.70}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_any_tripped());
}

// ==================== Reset Behavior Tests ====================

TEST_F(VhzProtectionTest, CanResetOnlyWhenFaultCleared)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10}, real_t{0.05});  // 5% hysteresis
    protection.configure_over_flux_trip_delay(duration_t{0.01});

    // Trip the protection
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_over_flux_tripped());

    // Fault still active → cannot reset
    EXPECT_FALSE(protection.can_reset());
    protection.reset();
    EXPECT_TRUE(protection.is_over_flux_tripped());  // Still tripped

    // Return to safe condition (below reset threshold with hysteresis)
    // Reset threshold = 1.10 * (1 - 0.05) = 1.045 pu
    run_steps(10, voltage_pu_t{1.04}, frequency_t{50.0});
    EXPECT_TRUE(protection.can_reset());

    // Now reset should clear the trip
    protection.reset();
    EXPECT_FALSE(protection.is_over_flux_tripped());
}

TEST_F(VhzProtectionTest, CannotResetIfEitherFaultActive)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80});
    protection.configure_under_flux_trip_delay(duration_t{0.01});

    // Trip both protections
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});  // Over-flux
    EXPECT_TRUE(protection.is_over_flux_tripped());

    // Change to under-flux condition (over-flux clears, under-flux trips)
    run_steps(100, voltage_pu_t{0.70}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_under_flux_tripped());

    // Even though over-flux condition cleared, cannot reset due to under-flux
    EXPECT_FALSE(protection.can_reset());

    // Return to safe mid-range condition
    run_steps(100, voltage_pu_t{0.95}, frequency_t{50.0});
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(VhzProtectionTest, ResetClearsBothTrips)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});

    // Trip protection
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_over_flux_tripped());
    EXPECT_TRUE(protection.is_any_tripped());

    // Clear fault and reset
    run_steps(10, voltage_pu_t{1.0}, frequency_t{50.0});
    protection.reset();

    EXPECT_FALSE(protection.is_over_flux_tripped());
    EXPECT_FALSE(protection.is_under_flux_tripped());
    EXPECT_FALSE(protection.is_any_tripped());
}

// ==================== Hysteresis Tests ====================

TEST_F(VhzProtectionTest, OverFluxHysteresisPreventsFalseReset)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10}, real_t{0.05});  // 5% hysteresis
    protection.configure_over_flux_trip_delay(duration_t{0.01});

    // Trip threshold: 1.10 pu
    // Reset threshold: 1.10 * (1 - 0.05) = 1.045 pu

    // Trip the protection
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_over_flux_tripped());

    // Reduce V/Hz to just below trip threshold (1.09 pu) → still cannot reset
    run_steps(10, voltage_pu_t{1.09}, frequency_t{50.0});
    EXPECT_FALSE(protection.can_reset());

    // Reduce V/Hz to below reset threshold (1.04 pu) → can reset
    run_steps(10, voltage_pu_t{1.04}, frequency_t{50.0});
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(VhzProtectionTest, UnderFluxHysteresisPreventsFalseReset)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_under_flux_trip_threshold(flux_pu_t{0.80}, real_t{0.05});  // 5% hysteresis
    protection.configure_under_flux_trip_delay(duration_t{0.01});

    // Trip threshold: 0.80 pu
    // Reset threshold: 0.80 * (1 + 0.05) = 0.84 pu

    // Trip the protection
    run_steps(100, voltage_pu_t{0.70}, frequency_t{50.0});
    EXPECT_TRUE(protection.is_under_flux_tripped());

    // Increase V/Hz to just above trip threshold (0.81 pu) → still cannot reset
    run_steps(10, voltage_pu_t{0.81}, frequency_t{50.0});
    EXPECT_FALSE(protection.can_reset());

    // Increase V/Hz to above reset threshold (0.85 pu) → can reset
    run_steps(10, voltage_pu_t{0.85}, frequency_t{50.0});
    EXPECT_TRUE(protection.can_reset());
}

// ==================== Transient Immunity Tests ====================

TEST_F(VhzProtectionTest, BriefOverFluxDoesNotTrip)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});  // 100 steps

    // Brief over-flux (50 steps) followed by normal operation
    run_steps(50, voltage_pu_t{1.15}, frequency_t{50.0});
    run_steps(100, voltage_pu_t{1.0}, frequency_t{50.0});

    EXPECT_FALSE(protection.is_over_flux_tripped());
}

TEST_F(VhzProtectionTest, PersistentOverFluxTripsAfterDelay)
{
    protection.configure_rated_vhz(flux_pu_t{1.0 / 50.0});
    protection.configure_over_flux_trip_threshold(flux_pu_t{1.10});
    protection.configure_over_flux_trip_delay(duration_t{0.01});  // 100 steps

    // Persistent over-flux for full delay
    run_steps(100, voltage_pu_t{1.15}, frequency_t{50.0});

    EXPECT_TRUE(protection.is_over_flux_tripped());
}
