#include <gtest/gtest.h>

#include "yeet_control/reference_frame.hpp"

namespace {

using namespace yeet;

static constexpr real_t k_epsilon = 1e-6;

TEST(LineVoltageTest, ConvertsFromPhaseToLine)
{
    const abc<real_t> phase{1.0, -0.5, -0.5};

    const line_voltage<real_t> line{phase};

    EXPECT_NEAR(line.ab(), 1.5, k_epsilon);
    EXPECT_NEAR(line.bc(), 0.0, k_epsilon);
    EXPECT_NEAR(line.ca(), -1.5, k_epsilon);
}

TEST(LineVoltageTest, ConvertsFromLineToPhaseBalanced)
{
    const abc<real_t> phase{2.0, -1.0, -1.0};

    const line_voltage<real_t> line{phase};
    const abc<real_t> recovered = line.to_phase_balanced();

    EXPECT_NEAR(recovered.a(), phase.a(), k_epsilon);
    EXPECT_NEAR(recovered.b(), phase.b(), k_epsilon);
    EXPECT_NEAR(recovered.c(), phase.c(), k_epsilon);
}

TEST(LineVoltageTest, HelperToLineVoltageMatchesConstructor)
{
    const abc<real_t> phase{0.2, -0.1, -0.1};

    const line_voltage<real_t> a{phase};
    const line_voltage<real_t> b = to_line_voltage(phase);

    EXPECT_NEAR(a.ab(), b.ab(), k_epsilon);
    EXPECT_NEAR(a.bc(), b.bc(), k_epsilon);
    EXPECT_NEAR(a.ca(), b.ca(), k_epsilon);
}

}  // namespace
