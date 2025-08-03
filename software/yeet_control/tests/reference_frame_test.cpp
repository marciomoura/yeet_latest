#include "yeet_control/reference_frame.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

static constexpr double k_epsilon = 1e-8;

// Helper functions to compare values within epsilon
void expect_near(double a, double b) { ASSERT_NEAR(a, b, k_epsilon); }

template <size_t n>
void expect_array_near(const std::array<double, n>& a, const std::array<double, n>& b)
{
    for (size_t i = 0; i < n; ++i) {
        ASSERT_NEAR(a[i], b[i], k_epsilon) << "Arrays differ at index " << i;
    }
}

// Common test values
const std::array<double, 3> balanced_abc = {1.0, -0.5, -0.5};
const std::array<double, 2> expected_alphabeta = {1.0, 0.0};
const std::array<double, 2> expected_dq = {1.0, 0.0};

// ABC Tests
TEST(ReferenceFrameTest, AbcDefaultConstructor)
{
    abc<double> coords;
    expect_array_near(coords.array(), {0.0, 0.0, 0.0});
}

TEST(ReferenceFrameTest, AbcArrayConstructor)
{
    abc<double> coords(balanced_abc);
    expect_array_near(coords.array(), balanced_abc);
}

TEST(ReferenceFrameTest, AbcValueConstructor)
{
    abc<double> coords(1.0, -0.5, -0.5);
    expect_array_near(coords.array(), balanced_abc);
}

TEST(ReferenceFrameTest, AbcArrayAccess)
{
    abc<double> coords(balanced_abc);
    expect_near(coords[0], 1.0);
    expect_near(coords[1], -0.5);
    expect_near(coords[2], -0.5);
}

TEST(ReferenceFrameTest, AbcToAlphabeta)
{
    abc<double> abc_coords(balanced_abc);
    auto ab_coords = abc_coords.to_alphabeta();
    expect_array_near(ab_coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, AbcAdditionAndSubtractionOperators)
{
    abc<double> a{1.0, 2.0, 3.0};
    abc<double> b{4.0, 5.0, 6.0};

    abc<double> sum = a + b;
    expect_near(sum[0], 5.0);
    expect_near(sum[1], 7.0);
    expect_near(sum[2], 9.0);

    abc<double> diff = b - a;
    expect_near(diff[0], 3.0);
    expect_near(diff[1], 3.0);
    expect_near(diff[2], 3.0);
}

TEST(ReferenceFrameTest, AbcUnaryNegationOperator)
{
    abc<double> abc_coords{1.0, -0.5, -0.5};
    abc<double> neg_abc = -abc_coords;
    expect_near(neg_abc[0], -1.0);
    expect_near(neg_abc[1], 0.5);
    expect_near(neg_abc[2], 0.5);
}

TEST(ReferenceFrameTest, AbcCompoundOperators)
{
    abc<double> a(1.0, 2.0, 3.0);
    abc<double> b(4.0, 5.0, 6.0);

    a += b;
    expect_array_near(a.array(), std::array<double, 3>{5.0, 7.0, 9.0});

    a -= b;
    expect_array_near(a.array(), std::array<double, 3>{1.0, 2.0, 3.0});

    a *= 2.0;
    expect_array_near(a.array(), std::array<double, 3>{2.0, 4.0, 6.0});

    a /= 2.0;
    expect_array_near(a.array(), std::array<double, 3>{1.0, 2.0, 3.0});
}

TEST(ReferenceFrameTest, AbcScalarMultiplicationAndDivision)
{
    abc<double> a(1.0, 2.0, 3.0);

    abc<double> b = a * 3.0;
    expect_array_near(b.array(), std::array<double, 3>{3.0, 6.0, 9.0});

    abc<double> c = a / 2.0;
    expect_array_near(c.array(), std::array<double, 3>{0.5, 1.0, 1.5});
}

TEST(ReferenceFrameTest, AbcIteratorAndSize)
{
    abc<double> coords{1.0, 2.0, 3.0};

    // Test size
    EXPECT_EQ(coords.size(), 3);

    // Test non-const iterator
    std::vector<double> values;
    for (auto& value : coords) {
        values.push_back(value);
    }
    EXPECT_EQ(values.size(), 3);
    EXPECT_NEAR(values[0], 1.0, k_epsilon);
    EXPECT_NEAR(values[1], 2.0, k_epsilon);
    EXPECT_NEAR(values[2], 3.0, k_epsilon);

    // Test const iterator
    const abc<double> const_coords{1.0, 2.0, 3.0};
    values.clear();
    for (const auto& value : const_coords) {
        values.push_back(value);
    }
    EXPECT_EQ(values.size(), 3);
    EXPECT_NEAR(values[0], 1.0, k_epsilon);
    EXPECT_NEAR(values[1], 2.0, k_epsilon);
    EXPECT_NEAR(values[2], 3.0, k_epsilon);
}

// Alphabeta Tests
TEST(ReferenceFrameTest, AlphabetaDefaultConstructor)
{
    alphabeta<double> coords;
    expect_array_near(coords.array(), {0.0, 0.0});
}

TEST(ReferenceFrameTest, AlphabetaArrayConstructor)
{
    alphabeta<double> coords(expected_alphabeta);
    expect_array_near(coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, AlphabetaValueConstructor)
{
    alphabeta<double> coords(1.0, 0.0);
    expect_array_near(coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, ConversionAlphaBetaToAbc)
{
    alphabeta<double> ab_coords(expected_alphabeta);
    auto abc_coords = ab_coords.to_abc();
    expect_array_near(abc_coords.array(), balanced_abc);
}

TEST(ReferenceFrameTest, ConversionAlphaBetaToDq)
{
    alphabeta<double> ab_coords(expected_alphabeta);
    auto dq_coords = ab_coords.to_dq(angle_wrapped(0.0));
    expect_array_near(dq_coords.array(), expected_dq);
}

TEST(ReferenceFrameTest, AlphabetaMagnitudeAndPhase)
{
    alphabeta<double> coords(1.0, 1.0);
    expect_near(coords.magnitude(), std::sqrt(2.0));
    expect_near(coords.phase().get_radians(), pi / 4);
}

TEST(ReferenceFrameTest, DqMagnitudeAndPHase)
{
    dq<double> coords(1.0, 1.0);
    expect_near(coords.magnitude(), std::sqrt(2.0));
    expect_near(coords.phase(angle_wrapped(0.0)).get_radians(), pi / 4);
}

TEST(ReferenceFrameTest, AlphabetaAdditionAndSubtractionOperators)
{
    alphabeta<double> a{1.0, 2.0};
    alphabeta<double> b{3.0, 4.0};

    alphabeta<double> sum = a + b;
    expect_near(sum[0], 4.0);
    expect_near(sum[1], 6.0);

    alphabeta<double> diff = b - a;
    expect_near(diff[0], 2.0);
    expect_near(diff[1], 2.0);
}

TEST(ReferenceFrameTest, AlphabetaUnaryNegationOperator)
{
    alphabeta<double> ab_coords{1.0, -0.5};
    alphabeta<double> neg_ab = -ab_coords;
    expect_near(neg_ab[0], -1.0);
    expect_near(neg_ab[1], 0.5);
}

TEST(ReferenceFrameTest, AlphabetaCompoundOperators)
{
    alphabeta<double> a(1.0, 2.0);
    alphabeta<double> b(3.0, 4.0);

    a += b;
    expect_array_near(a.array(), std::array<double, 2>{4.0, 6.0});

    a -= b;
    expect_array_near(a.array(), std::array<double, 2>{1.0, 2.0});

    a *= 2.0;
    expect_array_near(a.array(), std::array<double, 2>{2.0, 4.0});

    a /= 2.0;
    expect_array_near(a.array(), std::array<double, 2>{1.0, 2.0});
}

TEST(ReferenceFrameTest, AlphabetaScalarMultiplicationAndDivision)
{
    alphabeta<double> a(1.0, 2.0);

    alphabeta<double> b = a * 3.0;
    expect_array_near(b.array(), std::array<double, 2>{3.0, 6.0});

    alphabeta<double> c = a / 2.0;
    expect_array_near(c.array(), std::array<double, 2>{0.5, 1.0});
}

TEST(ReferenceFrameTest, AlphabetaIteratorAndSize)
{
    alphabeta<double> coords{1.0, 2.0};

    // Test size
    EXPECT_EQ(coords.size(), 2);

    // Test non-const iterator
    std::vector<double> values;
    for (auto& value : coords) {
        values.push_back(value);
    }
    EXPECT_EQ(values.size(), 2);
    EXPECT_NEAR(values[0], 1.0, k_epsilon);
    EXPECT_NEAR(values[1], 2.0, k_epsilon);

    // Test const iterator
    const alphabeta<double> const_coords{1.0, 2.0};
    values.clear();
    for (const auto& value : const_coords) {
        values.push_back(value);
    }
    EXPECT_EQ(values.size(), 2);
    EXPECT_NEAR(values[0], 1.0, k_epsilon);
    EXPECT_NEAR(values[1], 2.0, k_epsilon);
}

// DQ Tests
TEST(ReferenceFrameTest, DqDefaultConstructor)
{
    dq<double> coords;
    expect_array_near(coords.array(), {0.0, 0.0});
}

TEST(ReferenceFrameTest, DqArrayConstructor)
{
    dq<double> coords(expected_dq);
    expect_array_near(coords.array(), expected_dq);
}

TEST(ReferenceFrameTest, DqValueConstructor)
{
    dq<double> coords(1.0, 0.0);
    expect_array_near(coords.array(), expected_dq);
}

TEST(ReferenceFrameTest, DqToAlphabeta)
{
    dq<double> dq_coords(expected_dq);
    auto ab_coords = dq_coords.to_alphabeta(angle_wrapped(0.0));
    expect_array_near(ab_coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, DqMagnitude)
{
    dq<double> coords(3.0, 4.0);
    expect_near(coords.magnitude(), 5.0);
}

TEST(ReferenceFrameTest, DqAdditionAndSubtractionOperators)
{
    dq<double> a{1.0, 2.0};
    dq<double> b{3.0, 4.0};

    dq<double> sum = a + b;
    expect_near(sum[0], 4.0);
    expect_near(sum[1], 6.0);

    dq<double> diff = b - a;
    expect_near(diff[0], 2.0);
    expect_near(diff[1], 2.0);
}

TEST(ReferenceFrameTest, DqUnaryNegationOperator)
{
    dq<double> dq_coords{1.0, -0.5};
    dq<double> neg_dq = -dq_coords;
    expect_near(neg_dq[0], -1.0);
    expect_near(neg_dq[1], 0.5);
}

TEST(ReferenceFrameTest, DqCompoundOperators)
{
    dq<double> a(1.0, 2.0);
    dq<double> b(3.0, 4.0);

    a += b;
    expect_array_near(a.array(), std::array<double, 2>{4.0, 6.0});

    a -= b;
    expect_array_near(a.array(), std::array<double, 2>{1.0, 2.0});

    a *= 2.0;
    expect_array_near(a.array(), std::array<double, 2>{2.0, 4.0});

    a /= 2.0;
    expect_array_near(a.array(), std::array<double, 2>{1.0, 2.0});
}

TEST(ReferenceFrameTest, DqScalarMultiplicationAndDivision)
{
    dq<double> a(1.0, 2.0);

    dq<double> b = a * 3.0;
    expect_array_near(b.array(), std::array<double, 2>{3.0, 6.0});

    dq<double> c = a / 2.0;
    expect_array_near(c.array(), std::array<double, 2>{0.5, 1.0});
}

TEST(ReferenceFrameTest, DqIteratorAndSize)
{
    dq<double> coords{1.0, 2.0};

    // Test size
    EXPECT_EQ(coords.size(), 2);

    // Test non-const iterator
    std::vector<double> values;
    for (auto& value : coords) {
        values.push_back(value);
    }
    EXPECT_EQ(values.size(), 2);
    EXPECT_NEAR(values[0], 1.0, k_epsilon);
    EXPECT_NEAR(values[1], 2.0, k_epsilon);

    // Test const iterator
    const dq<double> const_coords{1.0, 2.0};
    values.clear();
    for (const auto& value : const_coords) {
        values.push_back(value);
    }
    EXPECT_EQ(values.size(), 2);
    EXPECT_NEAR(values[0], 1.0, k_epsilon);
    EXPECT_NEAR(values[1], 2.0, k_epsilon);
}

// Make functions tests
TEST(ReferenceFrameTest, MakeAbc)
{
    double magnitude = 1.0;
    auto coords = make_abc<double>(magnitude, angle_wrapped(0.0));

    // At 0 degrees, phase A should be at magnitude, B and C at -0.5*magnitude
    expect_near(coords[0], magnitude);
    expect_near(coords[1], -0.5 * magnitude);
    expect_near(coords[2], -0.5 * magnitude);
}

TEST(ReferenceFrameTest, MakeAlphabeta)
{
    double magnitude = 1.0;
    auto coords = make_alphabeta<double>(magnitude, angle_wrapped(0.0));

    // At 0 degrees, alpha should be magnitude, beta should be 0
    expect_near(coords[0], magnitude);
    expect_near(coords[1], 0.0);
}

// Full conversion cycle tests
TEST(ReferenceFrameTest, FullConversionCycle)
{
    // Start with abc
    abc<double> start_abc(balanced_abc);

    // Convert through all frames and back
    auto ab = start_abc.to_alphabeta();
    auto dq = ab.to_dq(angle_wrapped(0.0));
    auto back_ab = dq.to_alphabeta(angle_wrapped(0.0));
    abc<double> back_abc = back_ab.to_abc();
    ::dq<double> back_dq = back_abc.to_dq(angle_wrapped(0.0));

    // Should get back to original values
    expect_array_near(back_abc.array(), start_abc.array());
}

TEST(ReferenceFrameTest, NonMemberScalarMultiplication)
{
    abc<double> abc_coords(1.0, 2.0, 3.0);
    abc<double> abc_scaled = 2.0 * abc_coords;
    expect_array_near(abc_scaled.array(), std::array<double, 3>{2.0, 4.0, 6.0});

    alphabeta<double> ab_coords(1.0, 2.0);
    alphabeta<double> ab_scaled = 2.0 * ab_coords;
    expect_array_near(ab_scaled.array(), std::array<double, 2>{2.0, 4.0});

    dq<double> dq_coords(1.0, 2.0);
    dq<double> dq_scaled = 2.0 * dq_coords;
    expect_array_near(dq_scaled.array(), std::array<double, 2>{2.0, 4.0});
}

TEST(ReferenceFrameTest, AlphaBetaRotateCounterClockwise)
{
    alphabeta<double> ab = {1.0, 0.0};

    ab = ab.rotate_counter_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = 0, beta = 1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], 1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_counter_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = -1, beta = 0
    expect_near(ab[0], -1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_counter_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = 0, beta = -1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], -1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_counter_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = 1, beta = 0
    expect_near(ab[0], 1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);
}

TEST(ReferenceFrameTest, AlphaBetaRotateClockwise)
{
    alphabeta<double> ab = {1.0, 0.0};

    ab = ab.rotate_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = 0, beta = -1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], -1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = -1, beta = 0
    expect_near(ab[0], -1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = 0, beta = 1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], 1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_clockwise(angle_wrapped(pi / 2));
    // Result is alpha = 1, beta = 0
    expect_near(ab[0], 1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);
}

TEST(ReferenceFrameTest, DqRotateCounterClockwise)
{
    dq<double> dq_coords{1.0, 0.0};

    // Rotate counter-clockwise by 90 degrees (pi/2)
    dq<double> rotated = dq_coords.rotate_counter_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], 1.0);
    expect_near(rotated.magnitude(), 1.0);

    // Another 90° rotation (total 180°)
    rotated = rotated.rotate_counter_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], -1.0);
    expect_near(rotated[1], 0.0);

    // Another 90° rotation (total 270°)
    rotated = rotated.rotate_counter_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], -1.0);

    // Final 90° rotation should return to the original orientation
    rotated = rotated.rotate_counter_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], 1.0);
    expect_near(rotated[1], 0.0);
}

TEST(ReferenceFrameTest, DqRotateClockwise)
{
    dq<double> dq_coords{1.0, 0.0};

    // Rotate clockwise by 90 degrees (pi/2)
    dq<double> rotated = dq_coords.rotate_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], -1.0);
    expect_near(rotated.magnitude(), 1.0);

    // Another 90° rotation (total 180°)
    rotated = rotated.rotate_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], -1.0);
    expect_near(rotated[1], 0.0);

    // Another 90° rotation (total 270°)
    rotated = rotated.rotate_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], 1.0);

    // Final 90° rotation returns to original orientation
    rotated = rotated.rotate_clockwise(angle_wrapped(pi / 2));
    expect_near(rotated[0], 1.0);
    expect_near(rotated[1], 0.0);
}

TEST(ReferenceFrameTest, AlphaBetaToDQ)
{
    alphabeta<double> ab{1.0, 0.0};
    dq<double> dq = ab.to_dq(angle_wrapped(0.0));
    expect_near(dq[0], 1.0);
    expect_near(dq[1], 0.0);

    dq = ab.to_dq(angle_wrapped(pi / 2.0));
    expect_near(dq[0], 0.0);
    expect_near(dq[1], -1.0);

    dq = ab.to_dq(angle_wrapped(pi));
    expect_near(dq[0], -1.0);
    expect_near(dq[1], 0.0);

    dq = ab.to_dq(angle_wrapped(3 * pi / 2));
    expect_near(dq[0], 0.0);
    expect_near(dq[1], 1.0);
}

TEST(ReferenceFrameTest, DQToAlphaBeta)
{
    dq<double> dq{1.0, 0.0};
    alphabeta<double> ab = dq.to_alphabeta(angle_wrapped(0.0));
    expect_near(ab[0], 1.0);
    expect_near(ab[1], 0.0);

    ab = dq.to_alphabeta(angle_wrapped(pi / 2.0f));
    expect_near(ab[0], 0.0);
    expect_near(ab[1], 1.0);

    ab = dq.to_alphabeta(angle_wrapped(pi));
    expect_near(ab[0], -1.0);
    expect_near(ab[1], 0.0);

    ab = dq.to_alphabeta(angle_wrapped(3 * pi / 2));
    expect_near(ab[0], 0.0);
    expect_near(ab[1], -1.0);
}

TEST(ReferenceFrameTest, FullTransformation)
{
    abc<double> abc{1.0, -0.5, -0.5};
    alphabeta<double> ab = abc.to_alphabeta();
    dq<double> dq = ab.to_dq(angle_wrapped(0.0));
    alphabeta<double> back_ab = dq.to_alphabeta(angle_wrapped(0.0));
    ::abc<double> back_abc = back_ab.to_abc();
    ::dq<double> back_dq = back_abc.to_dq(angle_wrapped(0.0));

    expect_near(back_abc[0], abc[0]);
    expect_near(back_abc[1], abc[1]);
    expect_near(back_abc[2], abc[2]);

    expect_near(back_ab[0], ab[0]);
    expect_near(back_ab[1], ab[1]);

    expect_near(back_dq[0], dq[0]);
    expect_near(back_dq[1], dq[1]);
}

TEST(ReferenceFrameTest, ABCToAlphaBeta)
{
    // Original balanced example
    abc<double> abc_coords{1.0, -0.5, -0.5};
    alphabeta<double> ab_coords = abc_coords.to_alphabeta();
    expect_near(ab_coords[0], 1.0);
    expect_near(ab_coords[1], 0.0);

    // Test 45°: use make_abc to generate balanced abc with magnitude 1 and phase 45°
    {
        abc<double> abc45 = make_abc<double>(1.0, angle_wrapped(45.0 * pi / 180.0));
        alphabeta<double> ab45 = abc45.to_alphabeta();
        EXPECT_NEAR(ab45[0], std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab45[1], std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab45.magnitude(), 1.0, 1e-6);
    }

    // Test 90°: balanced abc corresponding to 90° (expected: alpha=0, beta=1)
    {
        abc<double> abc90 = make_abc<double>(1.0, angle_wrapped(90.0 * pi / 180.0));
        alphabeta<double> ab90 = abc90.to_alphabeta();
        EXPECT_NEAR(ab90[0], 0.0, 1e-6);
        EXPECT_NEAR(ab90[1], 1.0, 1e-6);
        EXPECT_NEAR(ab90.magnitude(), 1.0, 1e-6);
    }

    // Test 135°: balanced abc corresponding to 135° (expected: alpha ≈ -0.707107, beta ≈ 0.707107)
    {
        abc<double> abc135 = make_abc<double>(1.0, angle_wrapped(135.0 * pi / 180.0));
        alphabeta<double> ab135 = abc135.to_alphabeta();
        EXPECT_NEAR(ab135[0], -std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab135[1], std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab135.magnitude(), 1.0, 1e-6);
    }

    // Test 235°: generate balanced abc for 235° and verify both magnitude and phase.
    {
        abc<double> abc235 = make_abc<double>(1.0, angle_wrapped(235.0 * pi / 180.0));
        alphabeta<double> ab235 = abc235.to_alphabeta();
        EXPECT_NEAR(ab235.magnitude(), 1.0, 1e-6);
        // Convert phase (in radians) to degrees and adjust it into [0,360)
        double phase_deg = ab235.phase().get_radians() * 180.0 / pi;
        if (phase_deg < 0) phase_deg += 360.0;
        EXPECT_NEAR(phase_deg, 235.0, 1e-6);
    }
}

TEST(ReferenceFrameTest, AlphaBetaToABC)
{
    // Basic example: convert {1.0, 0.0} in αβ back to abc.
    {
        alphabeta<double> ab{1.0, 0.0};
        abc<double> abc_res = ab.to_abc();
        EXPECT_NEAR(abc_res[0], 1.0, 1e-6);
        EXPECT_NEAR(abc_res[1], -0.5, 1e-6);
        EXPECT_NEAR(abc_res[2], -0.5, 1e-6);
    }

    // Test 45°: Convert αβ (obtained from make_alphabeta) back to abc.
    {
        alphabeta<double> ab45 = make_alphabeta<double>(1.0, angle_wrapped(45.0 * pi / 180.0));
        abc<double> abc_from_ab = ab45.to_abc();
        // Expected abc obtained by make_abc should match the conversion.
        abc<double> expected_abc = make_abc<double>(1.0, angle_wrapped(45.0 * pi / 180.0));
        auto expected_arr = expected_abc.array();
        auto result_arr = abc_from_ab.array();
        for (size_t i = 0; i < 3; i++) {
            EXPECT_NEAR(result_arr[i], expected_arr[i], 1e-6);
        }
    }

    // Test 135°: Convert αβ from 135° back to abc.
    {
        alphabeta<double> ab135 = make_alphabeta<double>(1.0, angle_wrapped(135.0 * pi / 180.0));
        abc<double> abc_from_ab = ab135.to_abc();
        abc<double> expected_abc = make_abc<double>(1.0, angle_wrapped(135.0 * pi / 180.0));
        auto expected_arr = expected_abc.array();
        auto result_arr = abc_from_ab.array();
        for (size_t i = 0; i < 3; i++) {
            EXPECT_NEAR(result_arr[i], expected_arr[i], 1e-6);
        }
    }

    // Test 235°: Convert for 235°.
    {
        alphabeta<double> ab235 = make_alphabeta<double>(1.0, angle_wrapped(235.0 * pi / 180.0));
        abc<double> abc_from_ab = ab235.to_abc();
        abc<double> expected_abc = make_abc<double>(1.0, angle_wrapped(235.0 * pi / 180.0));
        auto expected_arr = expected_abc.array();
        auto result_arr = abc_from_ab.array();
        for (size_t i = 0; i < 3; i++) {
            EXPECT_NEAR(result_arr[i], expected_arr[i], 1e-6);
        }
    }
}

TEST(ReferenceFrameTest, IteratorModification)
{
    // Test abc modification through iterator
    abc<double> abc_coords{1.0, 2.0, 3.0};
    for (auto& value : abc_coords) {
        value *= 2.0;
    }
    EXPECT_NEAR(abc_coords[0], 2.0, k_epsilon);
    EXPECT_NEAR(abc_coords[1], 4.0, k_epsilon);
    EXPECT_NEAR(abc_coords[2], 6.0, k_epsilon);

    // Test alphabeta modification through iterator
    alphabeta<double> ab_coords{1.0, 2.0};
    for (auto& value : ab_coords) {
        value *= 2.0;
    }
    EXPECT_NEAR(ab_coords[0], 2.0, k_epsilon);
    EXPECT_NEAR(ab_coords[1], 4.0, k_epsilon);

    // Test dq modification through iterator
    dq<double> dq_coords{1.0, 2.0};
    for (auto& value : dq_coords) {
        value *= 2.0;
    }
    EXPECT_NEAR(dq_coords[0], 2.0, k_epsilon);
    EXPECT_NEAR(dq_coords[1], 4.0, k_epsilon);
}

class ReferenceFrameSimulatorTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;  // 10 kHz sampling rate
    static constexpr double signal_freq = 50.0;    // 50 Hz fundamental

    void SetUp() override
    {
        // simulator = yeet::testing::make_gtest_simulator();

        // Configure signal generator
        signal_gen->set_signal_frequency(signal_freq);
        signal_gen->set_fundamental_positive_sequence_signal_amplitude(1.0);

        // Register simulation updates
        simulator.register_lambda_runner(sampling_time, [&]() {
            // Update signal generator
            signal_gen->update();

            // Get ABC values
            abc_values = signal_gen->get_fundamental_signal_abc();

            // Transform to alpha-beta
            alphabeta_values = abc_values.to_alphabeta();

            // Calculate theta for dq transformation
            theta = alphabeta_values.phase();

            // Transform to dq
            dq_values = alphabeta_values.to_dq(theta);
        });

        // Register data sampling
        register_samplers();
    }

    bool simulate_for(double time) { return simulator.simulate_for(time); }

protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    std::unique_ptr<three_phase_waveform_generator> signal_gen =
        std::make_unique<three_phase_waveform_generator>(sampling_time);

    abc<double> abc_values{};
    alphabeta<double> alphabeta_values{};
    dq<double> dq_values{};
    angle_wrapped theta{0.0};

private:
    void register_samplers()
    {
        // Sample abc values
        simulator.register_sampler_entry(yeet::testing::data_sampler_entry("abc", abc_values));
        // Sample alpha-beta values
        simulator.register_sampler_entry(yeet::testing::data_sampler_entry("alphabeta", alphabeta_values));
        // Sample dq values
        simulator.register_sampler_entry(yeet::testing::data_sampler_entry("dq", dq_values));
        // Sample theta
        simulator.register_sampler_entry(
            yeet::testing::data_sampler_entry("theta", [this]() { return theta.get_pu(); }));
    }
};

TEST_F(ReferenceFrameSimulatorTest, BalancedTransformationChain)
{
    // For balanced signals:
    // - ABC should be 120° apart and equal magnitude
    // - Alpha-beta should have constant magnitude
    // - DQ should be constant (d ≈ 1, q ≈ 0)

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "d_value", [this]() { return dq_values[0]; }, 0.95, 1.05,  // Expect d ≈ 1.0 ±5%
        yeet::testing::checker_activation_time_range{0.1, 0.2}     // Check after initial settling
        ));

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "q_value", [this]() { return dq_values[1]; }, -0.05, 0.05,  // Expect q ≈ 0.0 ±5%
        yeet::testing::checker_activation_time_range{0.1, 0.2}));

    // Check alpha-beta magnitude is constant
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "alphabeta_magnitude", [this]() { return alphabeta_values.magnitude(); }, 0.95,
        1.05,  // Expect magnitude ≈ 1.0 ±5%
        yeet::testing::checker_activation_time_range{0.1, 0.2}));

    EXPECT_TRUE(simulate_for(0.2));  // Run for 10 cycles at 50 Hz
}

TEST_F(ReferenceFrameSimulatorTest, FrequencyStepResponse)
{
    // Test transformation chain during frequency step
    signal_gen->set_frequency_step(25.0, 0.1);  // Step to 75 Hz at t=0.1s

    // Check that DQ values remain constant even after frequency step
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "d_value_after_step", [this]() { return dq_values[0]; }, 0.95, 1.05,
        yeet::testing::checker_activation_time_range{0.15, 0.2}  // Check after step
        ));

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "q_value_after_step", [this]() { return dq_values[1]; }, -0.05, 0.05,
        yeet::testing::checker_activation_time_range{0.15, 0.2}));

    EXPECT_TRUE(simulate_for(0.2));  // Run for 10 cycles at 50 Hz
}

TEST_F(ReferenceFrameSimulatorTest, UnbalancedTransformationChain)
{
    // Test with unbalanced ABC (positive + negative sequence)
    signal_gen->set_fundamental_positive_sequence_signal_amplitude(0.9);  // 90% positive sequence
    signal_gen->set_fundamental_negative_sequence_signal_amplitude(0.1);  // 10% negative sequence

    // DQ should now have 100Hz oscillation due to negative sequence
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "d_oscillation", [this]() { return dq_values[0]; }, 0.8, 1.0,  // Expect d to oscillate between these values
        yeet::testing::checker_activation_time_range{0.1, 0.2}));

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "q_oscillation", [this]() { return dq_values[1]; }, -0.1, 0.1,  // Expect q to oscillate between these values
        yeet::testing::checker_activation_time_range{0.1, 0.2}));

    EXPECT_TRUE(simulate_for(0.2));  // Run for 10 cycles at 50 Hz
}

}  // namespace