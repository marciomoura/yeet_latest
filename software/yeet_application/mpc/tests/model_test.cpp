#include "mpc/model.h"

#include <gtest/gtest.h>

#include <vector>

#include "test_parameters.h"
#include "yeet_control/constants.hpp"

using namespace yeet::application::mpc;

struct OperatingPointAndExpectedMatrices {
    operating_point_type operating_point;
    Eigen::Matrix<double, 7, 3> expected_output_matrix;
    Eigen::Matrix<double, 3, 1> expected_input_matrix;
    Eigen::Matrix<double, 3, 3> expected_state_matrix;
};

class SynchronousMachineTiedToAnInfiniteBusModelTest : public ::testing::Test {
public:
    SynchronousMachineTiedToAnInfiniteBusModelTest() = default;

    ~SynchronousMachineTiedToAnInfiniteBusModelTest() = default;

    static constexpr double TWO_PI = 2.0 * yeet::control::pi<double>;

    machine_parameters machine;

    system_parameters system;

    void SetUp() override {
        machine = get_machine_parameters();

        system = get_system_parameters();

        model.set_machine_parameters(machine);

        model.set_system_parameters(system);

        {
            // This specific operating point has been obtained arbitrarily with the current Matlab/Simulink
            // implementation.
            Eigen::Matrix<double, 3, 3> const expected_state_matrix = get_state_matrix();

            Eigen::Matrix<double, 3, 1> const expected_input_matrix = get_input_matrix();

            Eigen::Matrix<double, 7, 3> const expected_output_matrix = get_output_matrix();

            operating_point_type const operating_point = get_operating_point();

            operating_points_and_expected_matrices.emplace_back(OperatingPointAndExpectedMatrices{
                operating_point, expected_output_matrix, expected_input_matrix, expected_state_matrix});
        }
    }

    void TearDown() override {}

    synchronous_machine_tied_to_infinite_bus_model model;

    std::vector<OperatingPointAndExpectedMatrices> operating_points_and_expected_matrices{};
};

TEST_F(SynchronousMachineTiedToAnInfiniteBusModelTest, Sum) {
    for (OperatingPointAndExpectedMatrices const& n : this->operating_points_and_expected_matrices) {
        this->model.update_model(n.operating_point);

        Eigen::Matrix<double, 3, 3> state_matrix = this->model.get_state_matrix();

        Eigen::Matrix<double, 3, 1> input_matrix = this->model.get_input_matrix();

        Eigen::Matrix<double, 7, 3> output_matrix = this->model.get_output_matrix();

        state_matrix.isApprox(n.expected_state_matrix, 1e-6);

        input_matrix.isApprox(n.expected_input_matrix, 1e-6);

        output_matrix.isApprox(n.expected_output_matrix, 1e-6);
    }
}
