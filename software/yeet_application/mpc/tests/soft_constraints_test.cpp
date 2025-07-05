#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "mpc/quadratic_program.h"

using namespace yeet::application::mpc;

class SoftConstraintsMatrixTest : public ::testing::Test {
protected:
    static constexpr std::size_t StatesN = 2;
    static constexpr std::size_t InputsN = 1;
    static constexpr std::size_t Np = 2;
    static constexpr std::size_t SoftConstraintsN = 2;

    void SetUp() override {
        // Setup system matrices with known values for easy verification
        a << 1.0, 0.2, 0.1, 1.0;

        b << 1.0, 0.5;

        // Setup constraint output matrices
        c[0] << 1.0, 0.0;  // First output (y1 = x1)
        c[1] << 0.0, 1.0;  // Second output (y2 = x2)

        // Setup constraints
        constraints[0] = {.actual = 1.0, .limit = 2.0, .type = soft_constraint_type::Max, .c = c[0]};

        constraints[1] = {.actual = -1.0, .limit = -0.5, .type = soft_constraint_type::Min, .c = c[1]};

        power_of_a = compute_powers_of_a<StatesN, Np>(a);

        // Pre-calculate some expected values
        expected_ab = a * b;
        expected_a2b = a * a * b;
    }

    // Helper functions to generate expected matrix blocks
    Eigen::Matrix<double, Np, Np * InputsN> calculate_expected_max_block(
        const Eigen::Matrix<double, 1, StatesN>& c_i) const {
        Eigen::Matrix<double, Np, Np * InputsN> block;
        // First row: C*B
        block(0, 0) = c_i * b;
        block(0, 1) = 0.0;
        // Second row: C*AB, C*B
        block(1, 0) = c_i * a * b;
        block(1, 1) = c_i * b;
        return block;
    }

    Eigen::Matrix<double, Np, Np * InputsN> calculate_expected_min_block(
        const Eigen::Matrix<double, 1, StatesN>& c_i) const {
        return -calculate_expected_max_block(c_i);
    }

    Eigen::Matrix<double, StatesN, StatesN> a;
    Eigen::Matrix<double, StatesN, InputsN> b;
    std::array<Eigen::Matrix<double, 1, StatesN>, SoftConstraintsN> c;
    std::array<SoftConstraint<StatesN>, SoftConstraintsN> constraints;
    std::array<Eigen::Matrix<double, StatesN, StatesN>, Np + 1> power_of_a;

    // Pre-calculated values
    Eigen::Matrix<double, StatesN, InputsN> expected_ab;
    Eigen::Matrix<double, StatesN, InputsN> expected_a2b;
};

// Verifies if the input constraint is triangular
TEST_F(SoftConstraintsMatrixTest, ValidateInputConstraintBlocks) {
    auto A = create_min_max_soft_constraints_a<StatesN, InputsN, Np, SoftConstraintsN>(a, b, constraints, power_of_a);

    // Check input upper bound block structure
    auto upper_block = A.block(0, 0, Np * InputsN, Np * InputsN);
    Eigen::Matrix<double, Np * InputsN, Np * InputsN> expected_upper;
    expected_upper << 1.0, 0.0, 1.0, 1.0;
    EXPECT_TRUE(upper_block.isApprox(expected_upper)) << "Input upper bound block incorrect\nExpected:\n"
                                                      << expected_upper << "\nGot:\n"
                                                      << upper_block;

    // Check input lower bound block structure
    auto lower_block = A.block(Np * InputsN, 0, Np * InputsN, Np * InputsN);
    EXPECT_TRUE(lower_block.isApprox(-expected_upper)) << "Input lower bound block incorrect\nExpected:\n"
                                                       << -expected_upper << "\nGot:\n"
                                                       << lower_block;
}

TEST_F(SoftConstraintsMatrixTest, ValidateMaxConstraintBlock) {
    auto A = create_min_max_soft_constraints_a<StatesN, InputsN, Np, SoftConstraintsN>(a, b, constraints, power_of_a);

    // Get the maximum constraint block
    auto max_block = A.block(2 * Np, 0, Np, Np * InputsN);

    // Calculate expected values using helper
    auto expected = calculate_expected_max_block(c[0]);

    EXPECT_TRUE(max_block.isApprox(expected)) << "Maximum constraint block incorrect\nExpected:\n"
                                              << expected << "\nGot:\n"
                                              << max_block;

    // Verify slack variable block
    auto max_slack = A.block(2 * Np, Np * InputsN, Np, Np);
    Eigen::Matrix<double, Np, Np> expected_slack;
    expected_slack << -1.0, 0.0, -1.0, -1.0;
    EXPECT_TRUE(max_slack.isApprox(expected_slack)) << "Maximum constraint slack block incorrect\nExpected:\n"
                                                    << expected_slack << "\nGot:\n"
                                                    << max_slack;
}

TEST_F(SoftConstraintsMatrixTest, ValidateMinConstraintBlock) {
    auto A = create_min_max_soft_constraints_a<StatesN, InputsN, Np, SoftConstraintsN>(a, b, constraints, power_of_a);

    // Get the minimum constraint block
    auto min_block = A.block(3 * Np, 0, Np, Np * InputsN);

    // Calculate expected values using helper
    auto expected = calculate_expected_min_block(c[1]);

    EXPECT_TRUE(min_block.isApprox(expected)) << "Minimum constraint block incorrect\nExpected:\n"
                                              << expected << "\nGot:\n"
                                              << min_block;

    // Verify slack variable block
    auto min_slack = A.block(3 * Np, 2 * Np, Np, Np);
    Eigen::Matrix<double, Np, Np> expected_slack;
    expected_slack << 1.0, 0.0, 1.0, 1.0;
    EXPECT_TRUE(min_slack.isApprox(expected_slack)) << "Minimum constraint slack block incorrect";
}

TEST_F(SoftConstraintsMatrixTest, ValidateConstraintBoundaries) {
    double u_current = 1.0;
    double u_max = 2.0;
    double u_min = 0.0;

    auto b =
        create_min_max_soft_constraints_b<StatesN, InputsN, SoftConstraintsN, Np>(u_current, u_max, u_min, constraints);

    // Check input constraint bounds
    std::vector<double> expected_input_max(Np, u_max - u_current);
    std::vector<double> expected_input_min(Np, -(u_min - u_current));

    for (size_t i = 0; i < Np; i++) {
        EXPECT_DOUBLE_EQ(b(i), expected_input_max[i]) << "Input max constraint boundary incorrect at index " << i;
        EXPECT_DOUBLE_EQ(b(Np + i), expected_input_min[i]) << "Input min constraint boundary incorrect at index " << i;
    }

    // Check soft constraint bounds
    for (size_t i = 0; i < SoftConstraintsN; i++) {
        double expected = constraints[i].type == soft_constraint_type::Max
                              ? (constraints[i].limit - constraints[i].actual)
                              : -(constraints[i].limit - constraints[i].actual);

        for (size_t j = 0; j < Np; j++) {
            EXPECT_DOUBLE_EQ(b(2 * Np + i * Np + j), expected)
                << "Soft constraint boundary incorrect for constraint " << i << " at prediction step " << j;
        }
    }
}
