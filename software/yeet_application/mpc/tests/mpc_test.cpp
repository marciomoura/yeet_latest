
// #include <gtest/gtest.h>

//
// #include <Eigen/Dense>
// #include <iostream>

// #include "mpc/model.h"
// #include "mpc/mpc.h"
// #include "test_parameters.h"

// using namespace yeet::application::mpc;

// TEST(MpcTest, HandlesGivenInput) {
//     mpc::excitation_mpc::excitation_mpc<50> mpc;

//     mpc.set_feedforward_output(1.0f);

//     mpc.set_parameters(get_machine_parameters(), get_system_parameters());

//     mpc.update(get_operating_point_mpc_test(), true);

//     // mpc.print_model();
//     // mpc.print_quadratic_program();
//     // std::cout << "expected hessian: \n" << get_hessian_matrix() << std::endl;
//     // mpc.print_gamma_ypsilon();

//     operating_point_type operating_point_type = get_operating_point_mpc_test();

//     operating_point.w0_pu = 1.0f;
//     operating_point.vt_0 = 1.0f;

//     mpc.update(operating_point, false);
//     std::cout << "Output 0: " << mpc.get_output() << std::endl;
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     mpc.update(operating_point, false);
//     std::cout << "Output 0: " << mpc.get_output() << std::endl;

//     operating_point.w0_pu = 1.0f;
//     operating_point.vt_0 = 1.0f;
//     mpc.update(operating_point, false);
//     std::cout << "Output 0: " << mpc.get_output() << std::endl;

//     operating_point.w0_pu = 0.98f;
//     operating_point.vt_0 = 1.0f;
//     mpc.update(operating_point, false);
//     std::cout << "Output 1: " << mpc.get_output() << std::endl;

//     operating_point.w0_pu = 1.0f;
//     operating_point.vt_0 = 1.0f;
//     mpc.update(operating_point, false);
//     std::cout << "Output 2: " << mpc.get_output() << std::endl;

//     operating_point.w0_pu = 1.0f;
//     operating_point.vt_0 = 0.98f;
//     mpc.update(operating_point, false);
//     std::cout << "Output 3: " << mpc.get_output() << std::endl;

//     operating_point.w0_pu = 1.0f;
//     operating_point.vt_0 = 1.0f;
//     mpc.update(operating_point, false);
//     std::cout << "Output 4: " << mpc.get_output() << std::endl;

//     operating_point.w0_pu = 1.0f;
//     operating_point.vt_0 = 1.0f;
//     mpc.update(operating_point, false);
//     std::cout << "Output 4: " << mpc.get_output() << std::endl;

//     operating_point.w0_pu = 1.0f;
//     operating_point.vt_0 = 1.0f;
//     mpc.update(operating_point, false);
//     std::cout << "Output 4: " << mpc.get_output() << std::endl;

//     mpc.set_terminal_voltage_reference(1.2f);
//     mpc.update(operating_point, false);
//     std::cout << "Output after voltage reference change: " << mpc.get_output() << std::endl;

//     EXPECT_EQ(1, 1);
// }