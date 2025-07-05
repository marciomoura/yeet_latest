#include "mpc/incremental_format.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

using namespace yeet::application::mpc;

TEST(StateSpaceForwardEulerDiscretizationTest, HandlesZeroInput) {
    static constexpr std::size_t StatesN = 3;
    static constexpr std::size_t InputsN = 2;

    Eigen::Matrix<double, StatesN, StatesN> f = Eigen::MatrixXd::Zero(StatesN, StatesN);
    Eigen::Matrix<double, StatesN, InputsN> g = Eigen::MatrixXd::Zero(StatesN, InputsN);

    double sampling_time = 0.1f;

    Eigen::Matrix<double, StatesN, StatesN> a;
    Eigen::Matrix<double, StatesN, InputsN> b;

    state_space_forward_euler_discretization<StatesN, InputsN>(f, g, sampling_time, a, b);

    EXPECT_TRUE(a.isApprox(Eigen::MatrixXd::Identity(StatesN, StatesN), 1e-6));
    EXPECT_TRUE(b.isApprox(Eigen::MatrixXd::Zero(StatesN, InputsN), 1e-6));
}

TEST(StateSpaceForwardEulerDiscretizationTest, HandlesModel) {
    static constexpr std::size_t StatesN = 3;
    static constexpr std::size_t InputsN = 2;

    Eigen::Matrix<double, StatesN, StatesN> f;
    Eigen::Matrix<double, StatesN, InputsN> g;

    f << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f;
    g << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f;

    double sampling_time = 0.1f;

    Eigen::Matrix<double, StatesN, StatesN> a;
    Eigen::Matrix<double, StatesN, InputsN> b;

    state_space_forward_euler_discretization<StatesN, InputsN>(f, g, sampling_time, a, b);

    Eigen::Matrix<double, StatesN, StatesN> expected_a;
    Eigen::Matrix<double, StatesN, InputsN> expected_b;

    expected_a << 1.1f, 0.2f, 0.3f, 0.4f, 1.5f, 0.6f, 0.7f, 0.8f, 1.9f;
    expected_b << 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f;

    EXPECT_TRUE(a.isApprox(expected_a, 1e-6));
    EXPECT_TRUE(b.isApprox(expected_b, 1e-6));
}

TEST(IncrementalFormatTest, HandlesIncrementalFormat) {
    const std::size_t StatesN = 3;
    const std::size_t InputsN = 1;
    const std::size_t OutputsN = 2;

    // Define the input matrices
    Eigen::Matrix<double, StatesN, StatesN> a{};
    Eigen::Matrix<double, StatesN, InputsN> b{};
    Eigen::Matrix<double, OutputsN, StatesN> c{};

    a << 0.998571428571429, -0.00354193363712181, -0.00223434939091615, 7.5398223686155, 1, 0, 0, -0.00282083493834186,
        0.994074433186277;

    b << 0, 0, 0.003;

    c << 0, 0.0312828872257362, 0.105103623559344, 1, 0, 0;

    // Define the expected output matrices
    Eigen::Matrix<double, StatesN + OutputsN, StatesN + OutputsN> expected_a_incremental{};
    Eigen::Matrix<double, StatesN + OutputsN, InputsN> expected_b_incremental{};
    Eigen::Matrix<double, OutputsN, StatesN + OutputsN> expected_c_incremental{};

    expected_a_incremental << 0.998571428571429, -0.00354193363712181, -0.00223434939091615, 0, 0, 7.5398223686155, 1,
        0, 0, 0, 0, -0.00282083493834186, 0.994074433186277, 0, 0, 0.235867412859482, 0.0309864072522536,
        0.104480825015578, 1, 0, 0.998571428571429, -0.00354193363712181, -0.00223434939091615, 0, 1;

    expected_b_incremental << 0, 0, 0.003, 0.000315310870678031, 0;

    expected_c_incremental << 0, 0, 0, 1.0f, 0,  //
        0, 0, 0, 0, 1.0f;

    // Call the function
    Eigen::Matrix<double, StatesN + OutputsN, StatesN + OutputsN> a_incremental{};
    Eigen::Matrix<double, StatesN + OutputsN, InputsN> b_incremental{};
    Eigen::Matrix<double, OutputsN, StatesN + OutputsN> c_incremental{};

    incremental_format<StatesN, InputsN, OutputsN>(a, b, c, a_incremental, b_incremental, c_incremental);

    // Check if the results are correct
    EXPECT_TRUE(a_incremental.isApprox(expected_a_incremental, 1e-6));
    EXPECT_TRUE(b_incremental.isApprox(expected_b_incremental, 1e-6));
    EXPECT_TRUE(c_incremental.isApprox(expected_c_incremental, 1e-6));
}
