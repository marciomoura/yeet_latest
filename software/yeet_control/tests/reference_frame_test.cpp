#include "yeet_control/reference_frame.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"

namespace {

using namespace yeet::control;

static constexpr double k_epsilon = 1e-8;

// Helper functions to compare values within epsilon
void expect_near(double a, double b) { ASSERT_NEAR(a, b, k_epsilon); }

template <size_t n>
void expect_array_near(const std::array<double, n>& a, const std::array<double, n>& b) {
    for (size_t i = 0; i < n; ++i) {
        ASSERT_NEAR(a[i], b[i], k_epsilon) << "Arrays differ at index " << i;
    }
}

// Common test values
const std::array<double, 3> balanced_abc = {1.0, -0.5, -0.5};
const std::array<double, 2> expected_alphabeta = {1.0, 0.0};
const std::array<double, 2> expected_dq = {1.0, 0.0};

// ABC Tests
TEST(ReferenceFrameTest, AbcDefaultConstructor) {
    abc<double> coords;
    expect_array_near(coords.array(), {0.0, 0.0, 0.0});
}

TEST(ReferenceFrameTest, AbcArrayConstructor) {
    abc<double> coords(balanced_abc);
    expect_array_near(coords.array(), balanced_abc);
}

TEST(ReferenceFrameTest, AbcValueConstructor) {
    abc<double> coords(1.0, -0.5, -0.5);
    expect_array_near(coords.array(), balanced_abc);
}

TEST(ReferenceFrameTest, AbcArrayAccess) {
    abc<double> coords(balanced_abc);
    expect_near(coords[0], 1.0);
    expect_near(coords[1], -0.5);
    expect_near(coords[2], -0.5);
}

TEST(ReferenceFrameTest, AbcToAlphabeta) {
    abc<double> abc_coords(balanced_abc);
    auto ab_coords = abc_coords.to_alphabeta();
    expect_array_near(ab_coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, AbcAdditionAndSubtractionOperators) {
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

TEST(ReferenceFrameTest, AbcUnaryNegationOperator) {
    abc<double> abc_coords{1.0, -0.5, -0.5};
    abc<double> neg_abc = -abc_coords;
    expect_near(neg_abc[0], -1.0);
    expect_near(neg_abc[1], 0.5);
    expect_near(neg_abc[2], 0.5);
}

TEST(ReferenceFrameTest, AbcCompoundOperators) {
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

TEST(ReferenceFrameTest, AbcScalarMultiplicationAndDivision) {
    abc<double> a(1.0, 2.0, 3.0);

    abc<double> b = a * 3.0;
    expect_array_near(b.array(), std::array<double, 3>{3.0, 6.0, 9.0});

    abc<double> c = a / 2.0;
    expect_array_near(c.array(), std::array<double, 3>{0.5, 1.0, 1.5});
}

// Alphabeta Tests
TEST(ReferenceFrameTest, AlphabetaDefaultConstructor) {
    alphabeta<double> coords;
    expect_array_near(coords.array(), {0.0, 0.0});
}

TEST(ReferenceFrameTest, AlphabetaArrayConstructor) {
    alphabeta<double> coords(expected_alphabeta);
    expect_array_near(coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, AlphabetaValueConstructor) {
    alphabeta<double> coords(1.0, 0.0);
    expect_array_near(coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, ConversionAlphaBetaToAbc) {
    alphabeta<double> ab_coords(expected_alphabeta);
    auto abc_coords = ab_coords.to_abc();
    expect_array_near(abc_coords.array(), balanced_abc);
}

TEST(ReferenceFrameTest, ConversionAlphaBetaToDq) {
    alphabeta<double> ab_coords(expected_alphabeta);
    auto dq_coords = ab_coords.to_dq(angle<zero_to_two_pi>(0.0));
    expect_array_near(dq_coords.array(), expected_dq);
}

TEST(ReferenceFrameTest, AlphabetaMagnitudeAndPhase) {
    alphabeta<double> coords(1.0, 1.0);
    expect_near(coords.magnitude(), std::sqrt(2.0));
    expect_near(coords.phase().get_radians(), pi<double> / 4);
}

TEST(ReferenceFrameTest, DqMagnitudeAndPHase) {
    dq<double> coords(1.0, 1.0);
    expect_near(coords.magnitude(), std::sqrt(2.0));
    expect_near(coords.phase(angle<zero_to_two_pi>(0.0)).get_radians(), pi<double> / 4);
}

TEST(ReferenceFrameTest, AlphabetaAdditionAndSubtractionOperators) {
    alphabeta<double> a{1.0, 2.0};
    alphabeta<double> b{3.0, 4.0};

    alphabeta<double> sum = a + b;
    expect_near(sum[0], 4.0);
    expect_near(sum[1], 6.0);

    alphabeta<double> diff = b - a;
    expect_near(diff[0], 2.0);
    expect_near(diff[1], 2.0);
}

TEST(ReferenceFrameTest, AlphabetaUnaryNegationOperator) {
    alphabeta<double> ab_coords{1.0, -0.5};
    alphabeta<double> neg_ab = -ab_coords;
    expect_near(neg_ab[0], -1.0);
    expect_near(neg_ab[1], 0.5);
}

TEST(ReferenceFrameTest, AlphabetaCompoundOperators) {
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

TEST(ReferenceFrameTest, AlphabetaScalarMultiplicationAndDivision) {
    alphabeta<double> a(1.0, 2.0);

    alphabeta<double> b = a * 3.0;
    expect_array_near(b.array(), std::array<double, 2>{3.0, 6.0});

    alphabeta<double> c = a / 2.0;
    expect_array_near(c.array(), std::array<double, 2>{0.5, 1.0});
}

// DQ Tests
TEST(ReferenceFrameTest, DqDefaultConstructor) {
    dq<double> coords;
    expect_array_near(coords.array(), {0.0, 0.0});
}

TEST(ReferenceFrameTest, DqArrayConstructor) {
    dq<double> coords(expected_dq);
    expect_array_near(coords.array(), expected_dq);
}

TEST(ReferenceFrameTest, DqValueConstructor) {
    dq<double> coords(1.0, 0.0);
    expect_array_near(coords.array(), expected_dq);
}

TEST(ReferenceFrameTest, DqToAlphabeta) {
    dq<double> dq_coords(expected_dq);
    auto ab_coords = dq_coords.to_alphabeta(angle<zero_to_two_pi>(0.0));
    expect_array_near(ab_coords.array(), expected_alphabeta);
}

TEST(ReferenceFrameTest, DqMagnitude) {
    dq<double> coords(3.0, 4.0);
    expect_near(coords.magnitude(), 5.0);
}

TEST(ReferenceFrameTest, DqAdditionAndSubtractionOperators) {
    dq<double> a{1.0, 2.0};
    dq<double> b{3.0, 4.0};

    dq<double> sum = a + b;
    expect_near(sum[0], 4.0);
    expect_near(sum[1], 6.0);

    dq<double> diff = b - a;
    expect_near(diff[0], 2.0);
    expect_near(diff[1], 2.0);
}

TEST(ReferenceFrameTest, DqUnaryNegationOperator) {
    dq<double> dq_coords{1.0, -0.5};
    dq<double> neg_dq = -dq_coords;
    expect_near(neg_dq[0], -1.0);
    expect_near(neg_dq[1], 0.5);
}

TEST(ReferenceFrameTest, DqCompoundOperators) {
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

TEST(ReferenceFrameTest, DqScalarMultiplicationAndDivision) {
    dq<double> a(1.0, 2.0);

    dq<double> b = a * 3.0;
    expect_array_near(b.array(), std::array<double, 2>{3.0, 6.0});

    dq<double> c = a / 2.0;
    expect_array_near(c.array(), std::array<double, 2>{0.5, 1.0});
}

// Make functions tests
TEST(ReferenceFrameTest, MakeAbc) {
    double magnitude = 1.0;
    auto coords = make_abc<double>(magnitude, angle<zero_to_two_pi>(0.0));

    // At 0 degrees, phase A should be at magnitude, B and C at -0.5*magnitude
    expect_near(coords[0], magnitude);
    expect_near(coords[1], -0.5 * magnitude);
    expect_near(coords[2], -0.5 * magnitude);
}

TEST(ReferenceFrameTest, MakeAlphabeta) {
    double magnitude = 1.0;
    auto coords = make_alphabeta<double>(magnitude, angle<zero_to_two_pi>(0.0));

    // At 0 degrees, alpha should be magnitude, beta should be 0
    expect_near(coords[0], magnitude);
    expect_near(coords[1], 0.0);
}

// Full conversion cycle tests
TEST(ReferenceFrameTest, FullConversionCycle) {
    // Start with abc
    abc<double> start_abc(balanced_abc);

    // Convert through all frames and back
    auto ab = start_abc.to_alphabeta();
    auto dq = ab.to_dq(angle<zero_to_two_pi>(0.0));
    auto back_ab = dq.to_alphabeta(angle<zero_to_two_pi>(0.0));
    abc<double> back_abc = back_ab.to_abc();
    ::dq<double> back_dq = back_abc.to_dq(angle<zero_to_two_pi>(0.0));

    // Should get back to original values
    expect_array_near(back_abc.array(), start_abc.array());
}

TEST(ReferenceFrameTest, NonMemberScalarMultiplication) {
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

TEST(ReferenceFrameTest, AlphaBetaRotateCounterClockwise) {
    alphabeta<double> ab = {1.0, 0.0};

    ab = ab.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = 0, beta = 1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], 1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = -1, beta = 0
    expect_near(ab[0], -1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = 0, beta = -1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], -1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = 1, beta = 0
    expect_near(ab[0], 1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);
}

TEST(ReferenceFrameTest, AlphaBetaRotateClockwise) {
    alphabeta<double> ab = {1.0, 0.0};

    ab = ab.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = 0, beta = -1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], -1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = -1, beta = 0
    expect_near(ab[0], -1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = 0, beta = 1
    expect_near(ab[0], 0.0);
    expect_near(ab[1], 1.0);
    expect_near(ab.magnitude(), 1.0);

    ab = ab.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    // Result is alpha = 1, beta = 0
    expect_near(ab[0], 1.0);
    expect_near(ab[1], 0.0);
    expect_near(ab.magnitude(), 1.0);
}

TEST(ReferenceFrameTest, DqRotateCounterClockwise) {
    dq<double> dq_coords{1.0, 0.0};

    // Rotate counter-clockwise by 90 degrees (pi/2)
    dq<double> rotated = dq_coords.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], 1.0);
    expect_near(rotated.magnitude(), 1.0);

    // Another 90° rotation (total 180°)
    rotated = rotated.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], -1.0);
    expect_near(rotated[1], 0.0);

    // Another 90° rotation (total 270°)
    rotated = rotated.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], -1.0);

    // Final 90° rotation should return to the original orientation
    rotated = rotated.rotate_counter_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], 1.0);
    expect_near(rotated[1], 0.0);
}

TEST(ReferenceFrameTest, DqRotateClockwise) {
    dq<double> dq_coords{1.0, 0.0};

    // Rotate clockwise by 90 degrees (pi/2)
    dq<double> rotated = dq_coords.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], -1.0);
    expect_near(rotated.magnitude(), 1.0);

    // Another 90° rotation (total 180°)
    rotated = rotated.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], -1.0);
    expect_near(rotated[1], 0.0);

    // Another 90° rotation (total 270°)
    rotated = rotated.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], 0.0);
    expect_near(rotated[1], 1.0);

    // Final 90° rotation returns to original orientation
    rotated = rotated.rotate_clockwise(angle<zero_to_two_pi>(pi<double> / 2));
    expect_near(rotated[0], 1.0);
    expect_near(rotated[1], 0.0);
}

TEST(ReferenceFrameTest, AlphaBetaToDQ) {
    alphabeta<double> ab{1.0, 0.0};
    dq<double> dq = ab.to_dq(angle<zero_to_two_pi>(0.0));
    expect_near(dq[0], 1.0);
    expect_near(dq[1], 0.0);

    dq = ab.to_dq(angle<zero_to_two_pi>(pi<double> / 2.0));
    expect_near(dq[0], 0.0);
    expect_near(dq[1], -1.0);

    dq = ab.to_dq(angle<zero_to_two_pi>(pi<double>));
    expect_near(dq[0], -1.0);
    expect_near(dq[1], 0.0);

    dq = ab.to_dq(angle<zero_to_two_pi>(3 * pi<double> / 2));
    expect_near(dq[0], 0.0);
    expect_near(dq[1], 1.0);
}

TEST(ReferenceFrameTest, DQToAlphaBeta) {
    dq<double> dq{1.0, 0.0};
    alphabeta<double> ab = dq.to_alphabeta(angle<zero_to_two_pi>(0.0));
    expect_near(ab[0], 1.0);
    expect_near(ab[1], 0.0);

    ab = dq.to_alphabeta(angle<zero_to_two_pi>(pi<double> / 2.0f));
    expect_near(ab[0], 0.0);
    expect_near(ab[1], 1.0);

    ab = dq.to_alphabeta(angle<zero_to_two_pi>(pi<double>));
    expect_near(ab[0], -1.0);
    expect_near(ab[1], 0.0);

    ab = dq.to_alphabeta(angle<zero_to_two_pi>(3 * pi<double> / 2));
    expect_near(ab[0], 0.0);
    expect_near(ab[1], -1.0);
}

TEST(ReferenceFrameTest, FullTransformation) {
    abc<double> abc{1.0, -0.5, -0.5};
    alphabeta<double> ab = abc.to_alphabeta();
    dq<double> dq = ab.to_dq(angle<zero_to_two_pi>(0.0));
    alphabeta<double> back_ab = dq.to_alphabeta(angle<zero_to_two_pi>(0.0));
    ::abc<double> back_abc = back_ab.to_abc();
    ::dq<double> back_dq = back_abc.to_dq(angle<zero_to_two_pi>(0.0));

    expect_near(back_abc[0], abc[0]);
    expect_near(back_abc[1], abc[1]);
    expect_near(back_abc[2], abc[2]);

    expect_near(back_ab[0], ab[0]);
    expect_near(back_ab[1], ab[1]);

    expect_near(back_dq[0], dq[0]);
    expect_near(back_dq[1], dq[1]);
}

TEST(ReferenceFrameTest, ABCToAlphaBeta) {
    // Original balanced example
    abc<double> abc_coords{1.0, -0.5, -0.5};
    alphabeta<double> ab_coords = abc_coords.to_alphabeta();
    expect_near(ab_coords[0], 1.0);
    expect_near(ab_coords[1], 0.0);

    // Test 45°: use make_abc to generate balanced abc with magnitude 1 and phase 45°
    {
        abc<double> abc45 = make_abc<double>(1.0, angle<zero_to_two_pi>(45.0 * pi<double> / 180.0));
        alphabeta<double> ab45 = abc45.to_alphabeta();
        EXPECT_NEAR(ab45[0], std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab45[1], std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab45.magnitude(), 1.0, 1e-6);
    }

    // Test 90°: balanced abc corresponding to 90° (expected: alpha=0, beta=1)
    {
        abc<double> abc90 = make_abc<double>(1.0, angle<zero_to_two_pi>(90.0 * pi<double> / 180.0));
        alphabeta<double> ab90 = abc90.to_alphabeta();
        EXPECT_NEAR(ab90[0], 0.0, 1e-6);
        EXPECT_NEAR(ab90[1], 1.0, 1e-6);
        EXPECT_NEAR(ab90.magnitude(), 1.0, 1e-6);
    }

    // Test 135°: balanced abc corresponding to 135° (expected: alpha ≈ -0.707107, beta ≈ 0.707107)
    {
        abc<double> abc135 = make_abc<double>(1.0, angle<zero_to_two_pi>(135.0 * pi<double> / 180.0));
        alphabeta<double> ab135 = abc135.to_alphabeta();
        EXPECT_NEAR(ab135[0], -std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab135[1], std::sqrt(2.0) / 2.0, 1e-6);
        EXPECT_NEAR(ab135.magnitude(), 1.0, 1e-6);
    }

    // Test 235°: generate balanced abc for 235° and verify both magnitude and phase.
    {
        abc<double> abc235 = make_abc<double>(1.0, angle<zero_to_two_pi>(235.0 * pi<double> / 180.0));
        alphabeta<double> ab235 = abc235.to_alphabeta();
        EXPECT_NEAR(ab235.magnitude(), 1.0, 1e-6);
        // Convert phase (in radians) to degrees and adjust it into [0,360)
        double phase_deg = ab235.phase().get_radians() * 180.0 / pi<double>;
        if (phase_deg < 0) phase_deg += 360.0;
        EXPECT_NEAR(phase_deg, 235.0, 1e-6);
    }
}

TEST(ReferenceFrameTest, AlphaBetaToABC) {
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
        alphabeta<double> ab45 = make_alphabeta<double>(1.0, angle<zero_to_two_pi>(45.0 * pi<double> / 180.0));
        abc<double> abc_from_ab = ab45.to_abc();
        // Expected abc obtained by make_abc should match the conversion.
        abc<double> expected_abc = make_abc<double>(1.0, angle<zero_to_two_pi>(45.0 * pi<double> / 180.0));
        auto expected_arr = expected_abc.array();
        auto result_arr = abc_from_ab.array();
        for (size_t i = 0; i < 3; i++) {
            EXPECT_NEAR(result_arr[i], expected_arr[i], 1e-6);
        }
    }

    // Test 135°: Convert αβ from 135° back to abc.
    {
        alphabeta<double> ab135 = make_alphabeta<double>(1.0, angle<zero_to_two_pi>(135.0 * pi<double> / 180.0));
        abc<double> abc_from_ab = ab135.to_abc();
        abc<double> expected_abc = make_abc<double>(1.0, angle<zero_to_two_pi>(135.0 * pi<double> / 180.0));
        auto expected_arr = expected_abc.array();
        auto result_arr = abc_from_ab.array();
        for (size_t i = 0; i < 3; i++) {
            EXPECT_NEAR(result_arr[i], expected_arr[i], 1e-6);
        }
    }

    // Test 235°: Convert for 235°.
    {
        alphabeta<double> ab235 = make_alphabeta<double>(1.0, angle<zero_to_two_pi>(235.0 * pi<double> / 180.0));
        abc<double> abc_from_ab = ab235.to_abc();
        abc<double> expected_abc = make_abc<double>(1.0, angle<zero_to_two_pi>(235.0 * pi<double> / 180.0));
        auto expected_arr = expected_abc.array();
        auto result_arr = abc_from_ab.array();
        for (size_t i = 0; i < 3; i++) {
            EXPECT_NEAR(result_arr[i], expected_arr[i], 1e-6);
        }
    }
}

}  // namespace