#include "yeet_control/virtual_angle_reference_generator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet;
using namespace yeet::control;

class VirtualAngleReferenceGeneratorTest : public ::testing::Test {
protected:
    virtual_angle_reference_generator gen;

    static constexpr double tolerance = 1e-9;
};

TEST_F(VirtualAngleReferenceGeneratorTest, InitialState)
{
    // Generator should start at zero angle
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, tolerance);

    // Default frequency should be 50 Hz
    EXPECT_NEAR(gen.get_frequency().value(), 50.0, tolerance);

    // Default sampling time should be 100 µs
    EXPECT_NEAR(gen.get_sampling_time().value(), 100e-6, tolerance);
}

TEST_F(VirtualAngleReferenceGeneratorTest, FrequencyConfiguration)
{
    gen.configure_frequency(frequency_t{60.0});
    EXPECT_NEAR(gen.get_frequency().value(), 60.0, tolerance);

    gen.configure_frequency(frequency_t{10.0});
    EXPECT_NEAR(gen.get_frequency().value(), 10.0, tolerance);
}

TEST_F(VirtualAngleReferenceGeneratorTest, SamplingTimeConfiguration)
{
    gen.configure_sampling_time(us(50.0));
    EXPECT_NEAR(gen.get_sampling_time().value(), 50e-6, tolerance);

    gen.configure_sampling_time(ms(1.0));
    EXPECT_NEAR(gen.get_sampling_time().value(), 1e-3, tolerance);
}

TEST_F(VirtualAngleReferenceGeneratorTest, BasicIntegration)
{
    // Configure for simple calculation: 50 Hz, 1 ms sampling
    gen.configure_frequency(frequency_t{50.0});
    gen.configure_sampling_time(ms(1.0));
    gen.reset();

    // After one update: angle = 2π × 50 × 0.001 = 0.1π radians
    gen.update();
    const double expected_angle_1 = two_pi * 50.0 * 1e-3;
    EXPECT_NEAR(gen.get_angle().get_radians().value(), expected_angle_1, tolerance);

    // After second update: angle = 0.2π radians
    gen.update();
    const double expected_angle_2 = two_pi * 50.0 * 2e-3;
    EXPECT_NEAR(gen.get_angle().get_radians().value(), expected_angle_2, tolerance);
}

TEST_F(VirtualAngleReferenceGeneratorTest, AngleWrapping)
{
    // Configure to wrap quickly: 1000 Hz, 1 ms sampling
    // Each update: angle += 2π × 1000 × 0.001 = 2π (exactly one full rotation)
    gen.configure_frequency(frequency_t{1000.0});
    gen.configure_sampling_time(ms(1.0));
    gen.reset();

    // After one update, should wrap back to ~0
    gen.update();
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, 1e-5);

    // Multiple updates should still wrap correctly
    for (int i = 0; i < 10; ++i) {
        gen.update();
    }
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, 1e-5);
}

TEST_F(VirtualAngleReferenceGeneratorTest, PartialWrapping)
{
    // Configure: 100 Hz, 1 ms sampling → 0.2π per update
    gen.configure_frequency(frequency_t{100.0});
    gen.configure_sampling_time(ms(1.0));
    gen.reset();

    // 5 updates → π radians
    for (int i = 0; i < 5; ++i) {
        gen.update();
    }
    EXPECT_NEAR(gen.get_angle().get_radians().value(), pi, tolerance);

    // 5 more updates → 2π (wraps to 0)
    for (int i = 0; i < 5; ++i) {
        gen.update();
    }
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, 1e-6);
}

TEST_F(VirtualAngleReferenceGeneratorTest, ResetToZero)
{
    gen.configure_frequency(frequency_t{50.0});
    gen.configure_sampling_time(ms(1.0));

    // Integrate to some non-zero value
    for (int i = 0; i < 10; ++i) {
        gen.update();
    }
    EXPECT_GT(gen.get_angle().get_radians().value(), 0.1);

    // Reset should return to zero
    gen.reset();
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, tolerance);
}

TEST_F(VirtualAngleReferenceGeneratorTest, ResetToBiasedValue)
{
    const angle_wrapped bias_angle = angle_wrapped::from_degrees(90.0);

    gen.reset(bias_angle);
    EXPECT_NEAR(gen.get_angle().get_degrees(), 90.0, tolerance);

    // Integration should continue from biased value
    gen.configure_frequency(frequency_t{50.0});
    gen.configure_sampling_time(ms(10.0));
    gen.update();

    const double expected_angle_deg = 90.0 + rad_to_deg(angle_t{two_pi * 50.0 * 10e-3});
    EXPECT_NEAR(gen.get_angle().get_degrees(), expected_angle_deg, 1e-6);
}

TEST_F(VirtualAngleReferenceGeneratorTest, ZeroFrequency)
{
    gen.configure_frequency(frequency_t{0.0});
    gen.configure_sampling_time(ms(1.0));
    gen.reset();

    // With zero frequency, angle should not change
    gen.update();
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, tolerance);

    for (int i = 0; i < 100; ++i) {
        gen.update();
    }
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, tolerance);
}

TEST_F(VirtualAngleReferenceGeneratorTest, HighFrequencyIntegration)
{
    // Test with high frequency and small time step
    gen.configure_frequency(frequency_t{10000.0});  // 10 kHz
    gen.configure_sampling_time(us(10.0));          // 10 µs
    gen.reset();

    // After 100 updates: angle = 2π × 10000 × 100 × 10e-6 = 2π (one full rotation)
    for (int i = 0; i < 100; ++i) {
        gen.update();
    }
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, 1e-4);
}

TEST_F(VirtualAngleReferenceGeneratorTest, LowFrequencyIntegration)
{
    // Test with low frequency
    gen.configure_frequency(frequency_t{1.0});  // 1 Hz
    gen.configure_sampling_time(ms(10.0));      // 10 ms
    gen.reset();

    // After 100 updates: 100 × 10ms = 1 second → one full cycle at 1 Hz
    for (int i = 0; i < 100; ++i) {
        gen.update();
    }
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, 1e-4);
}

TEST_F(VirtualAngleReferenceGeneratorTest, FrequencyChangesDuringOperation)
{
    gen.configure_frequency(frequency_t{50.0});
    gen.configure_sampling_time(ms(1.0));
    gen.reset();

    // Run at 50 Hz for 10 updates
    for (int i = 0; i < 10; ++i) {
        gen.update();
    }
    const double angle_after_50hz = gen.get_angle().get_radians().value();

    // Change to 100 Hz
    gen.configure_frequency(frequency_t{100.0});

    // Run at 100 Hz for 10 updates (should increment twice as fast)
    for (int i = 0; i < 10; ++i) {
        gen.update();
    }

    // The expected angle wraps around 2π automatically
    const double expected_increment_100hz = two_pi * 100.0 * 10e-3;
    const double expected_total_unwrapped = angle_after_50hz + expected_increment_100hz;

    // Need to manually wrap the expected value since it exceeds 2π
    const double expected_total_wrapped =
        expected_total_unwrapped - two_pi * std::floor(expected_total_unwrapped / two_pi);

    EXPECT_NEAR(gen.get_angle().get_radians().value(), expected_total_wrapped, 1e-6);
}

TEST_F(VirtualAngleReferenceGeneratorTest, RealisticGridFrequency)
{
    // Simulate realistic 50 Hz grid with 100 µs sampling (typical for power electronics)
    gen.configure_frequency(frequency_t{50.0});
    gen.configure_sampling_time(us(100.0));
    gen.reset();

    // Run for one full grid cycle (20 ms = 200 updates)
    for (int i = 0; i < 200; ++i) {
        gen.update();
    }

    // Should complete one full rotation (2π → 0)
    EXPECT_NEAR(gen.get_angle().get_radians().value(), 0.0, 1e-4);
}
