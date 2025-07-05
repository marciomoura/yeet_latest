#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <cmath>
#include <cstdint>

// #include <qpOASES.hpp>

namespace yeet::application::mpc {

/**
 * @brief Compute the powers of a state matrix efficiently
 *
 * Computes matrix powers from 0 to Np: [I, A, A^2, ..., A^Np]
 *
 * @tparam StatesN Number of state variables
 * @tparam Np Prediction horizon length
 * @param a State matrix to compute powers for
 * @return Array of matrix powers of dimension (Np+1)
 *
 * Matrix structure:
 * powers_of_a[0] = Identity matrix
 * powers_of_a[1] = A
 * powers_of_a[2] = A^2
 * ...
 * powers_of_a[Np] = A^Np
 */
template <std::size_t StatesN, std::size_t Np>
std::array<Eigen::Matrix<double, StatesN, StatesN>, Np + 1> compute_powers_of_a(
    Eigen::Matrix<double, StatesN, StatesN> const& a) {
    std::array<Eigen::Matrix<double, StatesN, StatesN>, Np + 1> powers_of_a{};

    // [Eye; A; A^2; ...; A^Np]
    powers_of_a[0] = Eigen::Matrix<double, StatesN, StatesN>::Identity();

    // Compute and store remaining powers
    for (std::uint32_t k = 1; k < Np + 1; ++k) {
        powers_of_a[k].noalias() = powers_of_a[k - 1] * a;
    }

    return powers_of_a;
}

/**
 * @brief Calculate prediction matrices Gamma and Ypsilon for Model Predictive Control
 *
 * Computes the matrices that expand the state-space model over a prediction horizon.
 * The resulting matrices satisfy: Y(k) = Gamma * x(k) + Ypsilon * U(k)
 *
 * @tparam StatesN Number of state variables
 * @tparam InputsN Number of input variables
 * @tparam OutputsN Number of output variables
 * @tparam Np Prediction horizon length
 *
 * @param a State transition matrix
 * @param b Input matrix
 * @param c Output matrix
 * @param powers_of_a Precomputed powers of the state matrix
 * @param[out] gamma Prediction matrix relating initial state to future outputs
 * @param[out] ypsilon Prediction matrix relating input sequence to future outputs
 *
 * Matrix structure for Np=3:
 * Gamma = [
 *   C*A,
 *   C*A^2,
 *   C*A^3
 * ]
 *
 * Ypsilon = [
 *   C*B    0       0
 *   C*A*B  C*B     0
 *   C*A^2*B C*A*B  C*B
 * ]
 */
template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN, std::size_t Np>
void calculate_gamma_and_ypsilon(Eigen::Matrix<double, StatesN, StatesN> const& a,
                                 Eigen::Matrix<double, StatesN, InputsN> const& b,
                                 Eigen::Matrix<double, OutputsN, StatesN> const& c,
                                 std::array<Eigen::Matrix<double, StatesN, StatesN>, Np + 1> const& powers_of_a,
                                 Eigen::Matrix<double, Np * OutputsN, StatesN>& gamma,
                                 Eigen::Matrix<double, Np * OutputsN, Np * InputsN>& ypsilon) {
    // Calculate Gamma using stored powers
    // Gamma = [CA; CA^2; CA^3; ...; CA^Np];
    for (std::uint32_t k = 0; k < Np; ++k) {
        gamma.template block<OutputsN, StatesN>(k * OutputsN, 0).noalias() = c * powers_of_a[k + 1];
    }

    // Ypsilon =[CB    0   0 ...; CAB   CB  0 ...; CA^2B CAB CB]
    using DiagonalBlock = Eigen::Matrix<double, OutputsN, InputsN>;
    DiagonalBlock diagonal{};
    for (std::uint32_t k = 0; k < Np; ++k) {
        diagonal.noalias() = c * powers_of_a[k] * b;
        for (std::uint32_t j = 0; j < (Np - k); ++j) {
            // This algorithm writes the main diagonal elements of the Ypsilon matrix
            // Then it jumps to write the next diagonal elements
            // Until it reaches one element to be written
            ypsilon.template block<OutputsN, InputsN>((k + j) * OutputsN, j * InputsN) = diagonal;
        }
    }
}

template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN, std::size_t Np>
void calculate_gamma_and_ypsilon(Eigen::Matrix<double, StatesN, StatesN> const& a,
                                 Eigen::Matrix<double, StatesN, InputsN> const& b,
                                 Eigen::Matrix<double, OutputsN, StatesN> const& c,
                                 Eigen::Matrix<double, Np * OutputsN, StatesN>& gamma,
                                 Eigen::Matrix<double, Np * OutputsN, Np * InputsN>& ypsilon) {
    std::array<Eigen::Matrix<double, StatesN, StatesN>, Np + 1> const powers_of_a = compute_powers_of_a<StatesN, Np>(a);

    calculate_gamma_and_ypsilon(a, b, c, powers_of_a, gamma, ypsilon);
}

/**
 * @brief Calculate quadratic term (Hessian) for Model Predictive Control optimization
 *
 * Computes the quadratic term H = Ypsilon^T * Q_tilda * Ypsilon + lambda * I
 *
 * @tparam StatesN Number of state variables
 * @tparam InputsN Number of input variables
 * @tparam OutputsN Number of output variables
 * @tparam Np Prediction horizon length
 *
 * @param ypsilon Prediction matrix from input sequence to outputs
 * @param q_tilda Sparse weighting matrix for tracking error
 * @param lambda Input penalization factor
 * @param[out] hessian Quadratic term matrix
 *
 * Matrix structure:
 * hessian = [
 *   Ypsilon^T * Q_tilda * Ypsilon + lambda * I
 * ]
 */
template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN, std::size_t Np,
          int StorageT = Eigen::RowMajor>
void calculate_quadratic_term(Eigen::Matrix<double, Np * OutputsN, Np * InputsN> const& ypsilon,
                              Eigen::DiagonalMatrix<double, Np * OutputsN> const& q_tilda, double lambda,
                              Eigen::Matrix<double, Np * InputsN, Np * InputsN, StorageT>& hessian) {
    hessian = ypsilon.transpose() * q_tilda * ypsilon +
              lambda * Eigen::Matrix<double, InputsN * Np, InputsN * Np>::Identity();
}

template <std::size_t OutputsN, std::size_t Np>
Eigen::DiagonalMatrix<double, Np * OutputsN> precompute_q_tilda(
    const Eigen::Matrix<double, OutputsN, 1>& penalization_q) {
    Eigen::Matrix<double, Np * OutputsN, 1> diagonal_values;

    for (std::uint32_t k = 0; k < Np; ++k) {
        for (std::uint32_t j = 0; j < OutputsN; ++j) {
            diagonal_values(k * OutputsN + j) = penalization_q(j);
        }
    }

    return diagonal_values.asDiagonal();
}

/**
 * @brief Calculate linear term (Theta) for MPC optimization
 *
 * Computes Theta = -((Y - Gamma * x0)^T * Q_tilda * Ypsilon)
 *
 * @tparam StatesN Number of state variables
 * @tparam InputsN Number of input variables
 * @tparam OutputsN Number of output variables
 * @tparam Np Prediction horizon length
 *
 * @param ypsilon Prediction matrix from input sequence to outputs
 * @param q_tilda Diagonal weighting matrix for tracking error
 * @param y Reference output trajectory
 * @param gamma Prediction matrix relating initial state to outputs
 * @param x0 Initial state
 * @param[out] theta Linear term vector
 */
template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN, std::size_t Np>
void calculate_linear_term(Eigen::Matrix<double, Np * OutputsN, Np * InputsN> const& ypsilon,
                           Eigen::DiagonalMatrix<double, Np * OutputsN> const& q_tilda,
                           Eigen::Matrix<double, Np * OutputsN, 1> const& y,
                           Eigen::Matrix<double, Np * OutputsN, StatesN> const& gamma,
                           Eigen::Matrix<double, StatesN, 1> const& x0, Eigen::Matrix<double, Np * InputsN, 1>& theta) {
    // Compute the linear term
    theta = -(y - gamma * x0).transpose() * q_tilda * ypsilon;
}

/**
 * @brief Augment the quadratic term with slack variables for soft constraints
 *
 * Adds penalization for soft constraint violations to the Hessian matrix
 *
 * @tparam StatesN Number of state variables
 * @tparam InputsN Number of input variables
 * @tparam OutputsN Number of output variables
 * @tparam Np Prediction horizon length
 * @tparam SoftConstraintsN Number of soft constraints
 *
 * @param hessian Original quadratic term matrix
 * @param soft_constraints_penalization_r Penalization factors for each soft constraint
 * @return Augmented Hessian matrix with slack variable penalties
 *
 * Matrix structure:
 * [
 *   Original Hessian   |   0
 *   0                  |   Diagonal(soft constraint penalties)
 * ]
 */
template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN, std::size_t Np, std::size_t SoftConstraintsN>
Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)>
augment_quadratic_term_with_slack_variables(Eigen::Matrix<double, Np * InputsN, Np * InputsN> const& hessian,
                                            std::array<double, SoftConstraintsN> soft_constraints_penalization_r) {
    Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)> augmented_hessian =
        Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)>::Zero();

    // Augment Hessian with soft constraints penalties
    augmented_hessian.topLeftCorner(Np * InputsN, Np * InputsN) = hessian;

    std::size_t offset = Np * InputsN;
    for (std::size_t i = 0; i < SoftConstraintsN; ++i) {
        augmented_hessian.block(offset, offset, Np, Np) =
            soft_constraints_penalization_r[i] * Eigen::Matrix<double, Np, Np>::Identity();

        // Move to next diagonal block
        offset += Np;
    }

    return augmented_hessian;
}

template <std::size_t SoftConstraintsN, std::size_t Np>
std::array<Eigen::Matrix<double, Np, 1>, SoftConstraintsN> create_horizon_weighted_penalties(double base_penalty) {
    std::array<Eigen::Matrix<double, Np, 1>, SoftConstraintsN> horizon_penalties;

    for (std::size_t i = 0; i < SoftConstraintsN; ++i) {
        horizon_penalties[i].setZero();

        // Quadratic decay of penalties across the horizon
        for (std::size_t j = 0; j < Np; ++j) {
            // Quadratic decay: higher penalty for early steps
            double weight = std::pow(1.0 - (j + 1.0) / Np, 2);
            horizon_penalties[i](j) = base_penalty * (1.0 + weight * 10.0);
        }
    }

    return horizon_penalties;
}

// Updated augment function
template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN, std::size_t Np, std::size_t SoftConstraintsN>
Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)>
augment_quadratic_term_with_horizon_weighted_slack_variables(
    Eigen::Matrix<double, Np * InputsN, Np * InputsN> const& hessian,
    std::array<Eigen::Matrix<double, Np, 1>, SoftConstraintsN> const& horizon_penalties) {
    Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)> augmented_hessian =
        Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)>::Zero();

    // Augment Hessian with soft constraints penalties
    augmented_hessian.topLeftCorner(Np * InputsN, Np * InputsN) = hessian;

    std::size_t offset = Np * InputsN;
    for (std::size_t i = 0; i < SoftConstraintsN; ++i) {
        // Create a diagonal matrix with horizon-weighted penalties
        augmented_hessian.block(offset, offset, Np, Np) = horizon_penalties[i].asDiagonal();

        // Move to next diagonal block
        offset += Np;
    }

    return augmented_hessian;
}

/**
 * @brief Augment the linear term with slack variables for soft constraints
 *
 * Extends the linear term vector to include slack variables
 *
 * @tparam StatesN Number of state variables
 * @tparam InputsN Number of input variables
 * @tparam OutputsN Number of output variables
 * @tparam Np Prediction horizon length
 * @tparam SoftConstraintsN Number of soft constraints
 *
 * @param theta Original linear term vector
 * @return Augmented linear term vector with zeros for slack variables
 *
 * Vector structure:
 * [
 *   Original Theta
 *   Zeros for slack variables
 * ]
 */
template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN, std::size_t Np, std::size_t SoftConstraintsN>
Eigen::Matrix<double, Np * InputsN + SoftConstraintsN * Np, 1> augment_linear_term_with_slack_variables(
    Eigen::Matrix<double, Np * InputsN, 1> const& theta) {
    Eigen::Matrix<double, Np * InputsN + SoftConstraintsN * Np, 1> augmented_theta =
        Eigen::Matrix<double, Np * InputsN + SoftConstraintsN * Np, 1>::Zero();

    augmented_theta.head(Np * InputsN) = theta;

    return augmented_theta;
}

namespace utils {

template <std::size_t Np, std::size_t InputsN, int StorageT = Eigen::RowMajor>
constexpr Eigen::Matrix<double, Np * InputsN, Np * InputsN, StorageT>
create_incremental_soft_constraints_triangular_matrix_a() {
    return Eigen::Matrix<double, Np * InputsN, Np * InputsN>::Ones().template triangularView<Eigen::Lower>();
}

}  // namespace utils

enum class soft_constraint_type { Max, Min };

template <std::size_t StatesN>
struct SoftConstraint {
    double actual{};
    double limit{};
    soft_constraint_type type{soft_constraint_type::Max};
    Eigen::Matrix<double, 1, StatesN> c{};
};

/**
 * @brief Create soft constraints matrix A for quadratic program
 *
 * Generates a comprehensive matrix for soft constraints across prediction horizon
 *
 * @tparam StatesN Number of state variables
 * @tparam InputsN Number of input variables
 * @tparam SoftConstraintsN Number of soft constraints
 * @tparam Np Prediction horizon length
 *
 * @param a State transition matrix
 * @param b Input matrix
 * @param soft_constraints Array of soft constraint configurations
 * @param power_of_a Precomputed powers of state matrix
 * @return Soft constraints matrix for QP
 *
 * Matrix structure (conceptual):
 * | Input upper bounds   |   Slack for Max  | Slack for Min   |
 * |----------------------|------------------|-----------------|
 * | 1  0                 |      0    0      |     0    0      |  Input max constraints
 * | 1  1                 |      0    0      |     0    0      |
 * |----------------------|------------------|-----------------|
 * |-1  0                 |      0    0      |     0    0      |  Input min constraints
 * |-1 -1                 |      0    0      |     0    0      |
 * |----------------------|------------------|-----------------|
 * | CB  0                |     -I    0      |     0    0      |  Max constraint on x1
 * | CAB CB               |     -I   -I      |     0    0      |
 * |----------------------|------------------|-----------------|
 * |-CB  0                |      0    0      |     I    0      |  Min constraint on x2
 * |-CAB -CB              |      0    0      |     I    I      |
 */
template <std::size_t StatesN, std::size_t InputsN, std::size_t SoftConstraintsN, std::size_t Np,
          int StorageT = Eigen::RowMajor>
constexpr Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), Np * InputsN * (1 + SoftConstraintsN), StorageT>
create_min_max_soft_constraints_a(Eigen::Matrix<double, StatesN, StatesN> const& a,
                                  Eigen::Matrix<double, StatesN, InputsN> const& b,
                                  std::array<SoftConstraint<StatesN>, SoftConstraintsN> const& soft_constraints,
                                  std::array<Eigen::Matrix<double, StatesN, StatesN>, Np + 1> const& power_of_a) {
    Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), Np * InputsN * (1 + SoftConstraintsN), StorageT>
        a_soft_constraints = Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN),
                                           Np * InputsN * (1 + SoftConstraintsN), StorageT>::Zero();

    // u < umax block (triangular)
    a_soft_constraints.block(0 * Np * InputsN, 0, Np * InputsN, Np * InputsN) =
        utils::create_incremental_soft_constraints_triangular_matrix_a<Np, InputsN, StorageT>();
    // -u < umin block (triangular)
    a_soft_constraints.block(1 * Np * InputsN, 0, Np * InputsN, Np * InputsN) =
        -utils::create_incremental_soft_constraints_triangular_matrix_a<Np, InputsN, StorageT>();

    for (std::size_t i = 0; i < SoftConstraintsN; ++i) {
        double const sign = soft_constraints[i].type == soft_constraint_type::Max ? -1.0f : 1.0f;

        a_soft_constraints.block((i + 2) * Np, (i + 1) * Np * InputsN, Np, Np) =
            sign * utils::create_incremental_soft_constraints_triangular_matrix_a<Np, InputsN, StorageT>();
    }

    for (std::size_t n = 0; n < Np; ++n) {
        for (std::size_t i = 0; i <= n; ++i) {
            auto const an_b = power_of_a[n - i] * b;

            for (std::size_t j = 0; j < SoftConstraintsN; ++j) {
                double const sign = soft_constraints[j].type == soft_constraint_type::Max ? 1.0f : -1.0f;
                auto value = sign * (soft_constraints[j].c * an_b);
                a_soft_constraints.template block<1, InputsN>(2 * Np + j * Np + n, i * InputsN) = value;
            }
        }
    }

    return a_soft_constraints;
}

/**
 * @brief Create soft constraints vector b for quadratic program
 *
 * Generates the right-hand side vector for soft constraints
 *
 * @tparam StatesN Number of state variables
 * @tparam InputsN Number of input variables
 * @tparam SoftConstraintsN Number of soft constraints
 * @tparam Np Prediction horizon length
 *
 * @param actual_u Current input value
 * @param max_u Maximum input limit
 * @param min_u Minimum input limit
 * @param soft_constraints Array of soft constraint configurations
 * @return Soft constraints vector for QP
 *
 * Vector structure:
 * [
 *   delta_umax * ones     // Input max constraints
 *   -delta_umin * ones    // Input min constraints
 *   Max/Min soft constraint limit differences
 * ]
 */
template <std::size_t StatesN, std::size_t InputsN, std::size_t SoftConstraintsN, std::size_t Np>
Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), InputsN> create_min_max_soft_constraints_b(
    double actual_u, double max_u, double min_u,
    std::array<SoftConstraint<StatesN>, SoftConstraintsN> const& soft_constraints) {
    double const delta_u_max = max_u - actual_u;
    double const delta_u_min = min_u - actual_u;

    Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), InputsN> min_max_soft_constraint =
        Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), InputsN>::Zero();

    min_max_soft_constraint.template topRows<Np * InputsN>() =
        delta_u_max * Eigen::Matrix<double, Np * InputsN, 1>::Ones();

    min_max_soft_constraint.template segment<Np * InputsN>(Np * InputsN) =
        -delta_u_min * Eigen::Matrix<double, Np * InputsN, 1>::Ones();

    for (std::size_t i = 0; i < SoftConstraintsN; ++i) {
        double const sign = soft_constraints[i].type == soft_constraint_type::Max ? 1.0f : -1.0f;

        min_max_soft_constraint.template segment<Np * InputsN>(2 * Np * InputsN + i * Np) =
            sign * (soft_constraints[i].limit - soft_constraints[i].actual) * Eigen::Matrix<double, Np, 1>::Ones();
    }

    return min_max_soft_constraint;
}

}  // namespace yeet::application::mpc
