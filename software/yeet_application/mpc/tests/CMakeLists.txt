project(yeet_application_mpc_tests CXX)

add_executable(${PROJECT_NAME})
add_executable(mpc::tests ALIAS ${PROJECT_NAME})

enable_testing()

target_sources(
    ${PROJECT_NAME}
    PRIVATE mpc_test.cpp
            model_test.cpp
            incremental_format_test.cpp
            quadratic_program_test.cpp
            soft_constraints_test.cpp
            test_parameters.h)

target_link_libraries(
    ${PROJECT_NAME} PRIVATE gtest::gtest_main yeet::yeet_control
                            yeet::yeet_application::mpc)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
