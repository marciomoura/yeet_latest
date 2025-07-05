#include "mpc/quadratic_program.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

using namespace yeet::application::mpc;

TEST(CalculateGammaAndYpsilonTest, HandlesGivenInput) {
    const std::size_t StatesN = 2;
    const std::size_t InputsN = 1;
    const std::size_t OutputsN = 2;
    const std::size_t Np = 3;

    // Define the input matrices
    Eigen::Matrix<double, StatesN, StatesN> a;
    a << 1, 2,  //
        3, 4;
    Eigen::Matrix<double, StatesN, InputsN> b;
    b << 0,  //
        2;
    Eigen::Matrix<double, OutputsN, StatesN> c;
    c << 1, 0,  //
        0, 1;

    // Define the expected output matrices
    Eigen::Matrix<double, Np * OutputsN, StatesN> expected_gamma =
        Eigen::Matrix<double, Np * OutputsN, StatesN>::Zero();

    expected_gamma << 1, 2,  //
        3, 4,                //
        7, 10,               //
        15, 22,              //
        37, 54,              //
        81, 118;             //

    Eigen::Matrix<double, Np * OutputsN, Np * InputsN> expected_ypsilon =
        Eigen::Matrix<double, Np * OutputsN, Np * InputsN>::Zero();

    expected_ypsilon << 0, 0, 0,  //
        2, 0, 0,                  //
        4, 0, 0,                  //
        8, 2, 0,                  //
        20, 4, 0,                 //
        44, 8, 2;

    // Call the function
    Eigen::Matrix<double, Np * OutputsN, StatesN> gamma = Eigen::Matrix<double, Np * OutputsN, StatesN>::Zero();

    Eigen::Matrix<double, Np * OutputsN, Np * InputsN> ypsilon =
        Eigen::Matrix<double, Np * OutputsN, Np * InputsN>::Zero();

    std::array<Eigen::Matrix<double, StatesN, StatesN>, Np + 1> power_of_a = compute_powers_of_a<StatesN, Np>(a);

    calculate_gamma_and_ypsilon<StatesN, InputsN, OutputsN, Np>(a, b, c, power_of_a, gamma, ypsilon);

    // Check if the results are correct
    EXPECT_TRUE(gamma.isApprox(expected_gamma, 1e-6));
    EXPECT_TRUE(ypsilon.isApprox(expected_ypsilon, 1e-6));
}

TEST(CalculateQuadraticTermTest, HandlesGivenInput) {
    const std::size_t StatesN = 2;
    const std::size_t InputsN = 1;
    const std::size_t OutputsN = 2;
    const std::size_t Np = 3;

    // Define the input matrices
    Eigen::Matrix<double, Np * OutputsN, Np * InputsN> ypsilon;
    ypsilon << 0, 0, 0,  //
        2, 0, 0,         //
        4, 0, 0,         //
        8, 2, 0,         //
        20, 4, 0,        //
        44, 8, 2;

    double lambda = 1e-4;

    // Define the expected output matrices
    Eigen::Matrix<double, Np * InputsN, Np * InputsN> expected_hessian =
        Eigen::Matrix<double, Np * InputsN, Np * InputsN>::Zero();

    expected_hessian << 2420.0001, 448, 88,  //
        448, 84.0001, 16,                    //
        88, 16, 4.0001;

    // Call the function
    Eigen::Matrix<double, Np * InputsN, Np * InputsN> hessian =
        Eigen::Matrix<double, Np * InputsN, Np * InputsN>::Zero();

    Eigen::Matrix<double, OutputsN, 1> penalization_q = Eigen::MatrixXd::Ones(OutputsN, 1);

    Eigen::DiagonalMatrix<double, Np * OutputsN> q_tilda = precompute_q_tilda<OutputsN, Np>(penalization_q);

    calculate_quadratic_term<StatesN, InputsN, OutputsN, Np>(ypsilon, q_tilda, lambda, hessian);

    // Check if the results are correct
    EXPECT_TRUE(hessian.isApprox(expected_hessian, 1e-6));
}

TEST(CalculateLinearTermTest, HandlesGivenInput) {
    const std::size_t StatesN = 2;
    const std::size_t InputsN = 1;
    const std::size_t OutputsN = 2;
    const std::size_t Np = 3;

    // Define the input matrices
    Eigen::Matrix<double, Np * OutputsN, Np * InputsN> ypsilon;
    ypsilon << 0, 0, 0,  //
        2, 0, 0,         //
        4, 0, 0,         //
        8, 2, 0,         //
        20, 4, 0,        //
        44, 8, 2;

    Eigen::Matrix<double, Np * OutputsN, 1> y;
    y << 1, 2, 3, 4, 5, 6;

    Eigen::Matrix<double, Np * OutputsN, StatesN> gamma;
    gamma << 1, 2,  //
        3, 4,       //
        7, 10,      //
        15, 22,     //
        37, 54,     //
        81, 118;    //

    Eigen::Matrix<double, StatesN, 1> x0;
    x0 << 0.5, 0.6;

    // Define the expected output matrices
    Eigen::Matrix<double, Np * InputsN, 1> expected_theta;
    expected_theta << 5714.6,  //
        1059.4,                //
        210.6;

    // Call the function
    Eigen::Matrix<double, Np * InputsN, 1> theta = Eigen::Matrix<double, Np * InputsN, 1>::Zero();

    Eigen::Matrix<double, OutputsN, 1> penalization_q = Eigen::MatrixXd::Ones(OutputsN, 1);

    Eigen::DiagonalMatrix<double, Np * OutputsN> q_tilda = precompute_q_tilda<OutputsN, Np>(penalization_q);

    calculate_linear_term<StatesN, InputsN, OutputsN, Np>(ypsilon, q_tilda, y, gamma, x0, theta);

    // Check if the results are correct
    EXPECT_TRUE(theta.isApprox(expected_theta, 1e-6));
}

TEST(QTildaTest, HandlesGivenInput) {
    const std::size_t StatesN = 2;
    const std::size_t InputsN = 1;
    const std::size_t OutputsN = 2;
    const std::size_t Np = 3;

    Eigen::Matrix<double, OutputsN, 1> penalization_q;
    penalization_q << 1.0f, 2.0f;

    Eigen::DiagonalMatrix<double, Np * OutputsN> q_tilda = precompute_q_tilda<OutputsN, Np>(penalization_q);

    EXPECT_FLOAT_EQ(q_tilda.diagonal()[0], 1.0f);
    EXPECT_FLOAT_EQ(q_tilda.diagonal()[1], 2.0f);
    EXPECT_FLOAT_EQ(q_tilda.diagonal()[2], 1.0f);
    EXPECT_FLOAT_EQ(q_tilda.diagonal()[3], 2.0f);
    EXPECT_FLOAT_EQ(q_tilda.diagonal()[4], 1.0f);
    EXPECT_FLOAT_EQ(q_tilda.diagonal()[5], 2.0f);
}

// TEST(SolveQuadraticProgramTest, HandlesGivenInput) {
//     const std::size_t StatesN = 3;
//     const std::size_t InputsN = 1;
//     const std::size_t Np = 5;

//     // Define the input matrices
//     Eigen::Matrix<qp_real, Np * InputsN, Np * InputsN> hessian;
//     hessian << 0.111541844921779, 0.007365112209524, 0.004082492935289, 0.001792842395930, 0.000485027439940,  //
//         0.007365112209524, 0.104924966224457, 0.002874660167402, 0.001340264651401, 0.000391658783168,         //
//         0.004082492935289, 0.002874660167402, 0.101816418922143, 0.000920853179782, 0.000295870628143,         //
//         0.001792842395930, 0.001340264651401, 0.000920853179782, 0.100539064540933, 0.000198252801302,         //
//         0.000485027439940, 0.000391658783168, 0.000295870628143, 0.000198252801302, 0.100099420963432;

//     hessian = hessian * 1e-3;

//     Eigen::Matrix<qp_real, Np * InputsN, 1> theta;
//     theta << -0.106470238769491, -0.062655137662220, -0.031008061705110, -0.011345130466936, -0.002088041918549;

//     theta = theta * 1e-3;

//     // Define the expected output matrices
//     Eigen::Matrix<qp_real, Np * InputsN, 1> expected_delta_u;

//     expected_delta_u << 0.909147892309648, 0.525243446307295, 0.252436424081276, 0.087290110951429,
//     0.013480283818713;

//     // Call the function
//     Eigen::Matrix<qp_real, Np * InputsN, 1> delta_u = Eigen::Matrix<qp_real, Np * InputsN, 1>::Zero();

//     solve_quadratic_program<StatesN, InputsN, Np>(hessian, theta, delta_u);

//     // Check if the results are correct
//     EXPECT_TRUE(delta_u.isApprox(expected_delta_u, 1e-6));

//     // std::cout << "hessian\n:" << hessian << std::endl;
//     // std::cout << "theta\n:" << theta << std::endl;
//     // std::cout << "delta_u\n:" << delta_u << std::endl;
//     // std::cout << "expected_delta_u\n:" << expected_delta_u << std::endl;
// }