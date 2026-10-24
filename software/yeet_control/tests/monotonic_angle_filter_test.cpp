#include "yeet_control/monotonic_angle_filter.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

namespace {

using namespace yeet::control;
using namespace yeet;

class MonotonicAngleFilterTest : public ::testing::Test {
protected:
    monotonic_angle_filter filter;
};

TEST_F(MonotonicAngleFilterTest, NormalForwardMovement)
{
    filter.reset();

    // Initial update sets the baseline
    filter.update(angle_wrapped(1.0f));
    EXPECT_NEAR(filter.get_output().get_radians(), 1.0f, 1e-5);

    // Forward movement should be reflected
    filter.update(angle_wrapped(1.1f));
    EXPECT_NEAR(filter.get_output().get_radians(), 1.1f, 1e-5);

    // Continue forward
    filter.update(angle_wrapped(1.5f));
    EXPECT_NEAR(filter.get_output().get_radians(), 1.5f, 1e-5);
}

TEST_F(MonotonicAngleFilterTest, BackwardJumpDetection)
{
    filter.reset();

    filter.update(angle_wrapped(1.0f));
    EXPECT_NEAR(filter.get_output().get_radians(), 1.0f, 1e-5);

    // Small backward jump (noise)
    filter.update(angle_wrapped(0.95f));
    // Should hold last valid position (1.0)
    EXPECT_NEAR(filter.get_output().get_radians(), 1.0f, 1e-5);

    // Large backward jump (noise)
    // By default max_backward_step is 60 deg (~1.047 rad).
    // A jump from 1.0 to 0.5 is -0.5 rad (~28 deg). This is inside the "stick" range.
    filter.update(angle_wrapped(0.5f));
    EXPECT_NEAR(filter.get_output().get_radians(), 1.0f, 1e-5);
}

TEST_F(MonotonicAngleFilterTest, UnstickOnLargeBackwardJump)
{
    filter.reset();
    filter.configure_max_backward_step(deg_to_rad(60.0f).value());

    filter.update(angle_wrapped(deg_to_rad(100.0f)));
    EXPECT_NEAR(filter.get_output().get_degrees(), 100.0f, 1e-4);

    // Small backward jump (-10 deg): Should stay stuck at 100
    filter.update(angle_wrapped(deg_to_rad(90.0f)));
    EXPECT_NEAR(filter.get_output().get_degrees(), 100.0f, 1e-4);

    // Large backward jump (-70 deg): Should unstick and follow to 30
    filter.update(angle_wrapped(deg_to_rad(30.0f)));
    EXPECT_NEAR(filter.get_output().get_degrees(), 30.0f, 1e-4);
}

TEST_F(MonotonicAngleFilterTest, ForwardAfterBackward)
{
    filter.reset();

    filter.update(angle_wrapped(1.0f));
    filter.update(angle_wrapped(0.95f));  // Backward noise
    EXPECT_NEAR(filter.get_output().get_radians(), 1.0f, 1e-5);

    // Moving forward again but still behind last valid
    filter.update(angle_wrapped(0.98f));
    EXPECT_NEAR(filter.get_output().get_radians(), 1.0f, 1e-5);

    // Moving forward past last valid
    filter.update(angle_wrapped(1.05f));
    EXPECT_NEAR(filter.get_output().get_radians(), 1.05f, 1e-5);
}

TEST_F(MonotonicAngleFilterTest, WrapAroundHandling)
{
    filter.reset();

    // Near 2PI
    filter.update(angle_wrapped(two_pi - 0.1f));
    EXPECT_NEAR(filter.get_output().get_radians(), two_pi - 0.1f, 1e-5);

    // Wrap over to small positive (Forward)
    filter.update(angle_wrapped(0.1f));
    EXPECT_NEAR(filter.get_output().get_radians(), 0.1f, 1e-5);

    // Backward jump across the wrap point
    filter.update(angle_wrapped(two_pi - 0.05f));
    // Should hold 0.1f
    EXPECT_NEAR(filter.get_output().get_radians(), 0.1f, 1e-5);
}

TEST_F(MonotonicAngleFilterTest, MaximumForwardJump)
{
    filter.reset();

    filter.update(angle_wrapped(0.0f));

    // Jump by exactly PI (Edge case of forward/backward decision)
    // 0 -> PI is exactly half a rotation
    filter.update(angle_wrapped(pi));
    EXPECT_NEAR(filter.get_output().get_radians(), pi, 1e-5);

    // Now try to jump from 0 to slightly more than PI
    filter.reset();
    filter.update(angle_wrapped(0.0f));

    // With default threshold (60 deg), a jump of PI + 0.01 (which is a backward jump
    // of PI - 0.01) is "large", so it follows.
    filter.update(angle_wrapped(pi + 0.01f));
    EXPECT_NEAR(filter.get_output().get_radians(), pi + 0.01f, 1e-5);

    // If we increase the threshold to 180 deg, it should ratchet again
    filter.reset();
    filter.configure_max_backward_step(pi);
    filter.update(angle_wrapped(0.0f));
    filter.update(angle_wrapped(pi + 0.01f));
    EXPECT_NEAR(filter.get_output().get_radians(), 0.0f, 1e-5);
}

TEST_F(MonotonicAngleFilterTest, ResetTest)
{
    filter.update(angle_wrapped(1.0f));
    filter.update(angle_wrapped(1.5f));

    filter.reset();

    // First update after reset should establish new baseline
    filter.update(angle_wrapped(0.5f));
    EXPECT_NEAR(filter.get_output().get_radians(), 0.5f, 1e-5);
}

}  // namespace
