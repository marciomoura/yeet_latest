#pragma once

#include <Eigen/Dense>

namespace yeet::application::mpc {

template <std::size_t StatesN, std::size_t InputsN>
void state_space_forward_euler_discretization(Eigen::Matrix<double, StatesN, StatesN> const& f,
                                              Eigen::Matrix<double, StatesN, InputsN> const& g, double sampling_time,
                                              Eigen::Matrix<double, StatesN, StatesN>& a,
                                              Eigen::Matrix<double, StatesN, InputsN>& b) {
    a = f * sampling_time + Eigen::Matrix<double, StatesN, StatesN>::Identity();
    b = g * sampling_time;
}

template <std::size_t StatesN, std::size_t InputsN>
void state_space_exact_discretization(Eigen::Matrix<double, StatesN, StatesN> const& f,
                                      Eigen::Matrix<double, StatesN, InputsN> const& g, double sampling_time,
                                      Eigen::Matrix<double, StatesN, StatesN>& a,
                                      Eigen::Matrix<double, StatesN, InputsN>& b) {
    // Compute A_d = exp(A * T)
    a = (f * sampling_time).exp();

    // Compute B_d = (A^-1 * (A_d - I)) * B
    Eigen::Matrix<double, StatesN, StatesN> identity = Eigen::Matrix<double, StatesN, StatesN>::Identity();
    if (f.determinant() != 0) {  // Check if F is invertible
        b = f.inverse() * (a - identity) * g;
    } else {
        // Fallback: Use numerical approximation or handle singular cases
        Eigen::Matrix<double, StatesN + InputsN, StatesN + InputsN> augmented;
        augmented.setZero();
        augmented.template topLeftCorner<StatesN, StatesN>() = f * sampling_time;
        augmented.template topRightCorner<StatesN, InputsN>() = g * sampling_time;

        Eigen::Matrix<double, StatesN + InputsN, StatesN + InputsN> exp_augmented = augmented.exp();
        a = exp_augmented.template topLeftCorner<StatesN, StatesN>();
        b = exp_augmented.template topRightCorner<StatesN, InputsN>();
    }
}

// Matrix augmentation
// This form is known as the velocity for model.
// New states are defined as z = [delta_x, y], given that
// delta_x is:
//
//       delta_x[k] = x[k]-x[k-1]
//       delta_u[k] = u[k]-u[k-1]
//
// We write the new system of matrices as:
//
//   x[k+1] = A x[k] + B delta_u[k]
//
//   A = [A, 0; CA, I]
//   B = [B; CB]
//   C = [0, I]
template <std::size_t StatesN, std::size_t InputsN, std::size_t OutputsN>
void incremental_format(Eigen::Matrix<double, StatesN, StatesN> const& a,
                        Eigen::Matrix<double, StatesN, InputsN> const& b,
                        Eigen::Matrix<double, OutputsN, StatesN> const& c,
                        Eigen::Matrix<double, StatesN + OutputsN, StatesN + OutputsN>& a_incremental,
                        Eigen::Matrix<double, StatesN + OutputsN, InputsN>& b_incremental,
                        Eigen::Matrix<double, OutputsN, StatesN + OutputsN>& c_incremental) {
    // A = [A, 0; CA, I]
    a_incremental.template topLeftCorner<StatesN, StatesN>() = a;
    a_incremental.template topRightCorner<StatesN, OutputsN>() = Eigen::Matrix<double, StatesN, OutputsN>::Zero();

    a_incremental.template bottomLeftCorner<OutputsN, StatesN>() = c * a;
    a_incremental.template bottomRightCorner<OutputsN, OutputsN>() =
        Eigen::Matrix<double, OutputsN, OutputsN>::Identity();

    // B = [B; CB]
    b_incremental.template topRows<StatesN>() = b;
    b_incremental.template bottomRows<OutputsN>() = c * b;

    // C = [0, I]
    c_incremental.setZero();
    c_incremental.template block<OutputsN, OutputsN>(0, StatesN).template triangularView<Eigen::Upper>() =
        Eigen::Matrix<double, OutputsN, OutputsN>::Identity();
}

template <std::size_t StatesN, std::size_t OutputsN>
Eigen::Matrix<double, StatesN + OutputsN, 1> incremental_format_x0(Eigen::Matrix<double, StatesN, 1> const& dx0_k,
                                                                   Eigen::Matrix<double, OutputsN, 1> const& y0) {
    Eigen::Matrix<double, StatesN + OutputsN, 1> incremental_format_x0;

    incremental_format_x0.template topRows<StatesN>() = dx0_k;
    incremental_format_x0.template bottomRows<OutputsN>() = y0;

    return incremental_format_x0;
}

}  // namespace yeet::application::mpc