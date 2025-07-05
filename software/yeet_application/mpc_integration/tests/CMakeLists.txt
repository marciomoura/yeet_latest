project(yeet_application_mpc_integration_tests CXX)

add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_application::mpc_integration_tests ALIAS
               ${PROJECT_NAME})

enable_testing()

target_sources(
    ${PROJECT_NAME}
    PRIVATE mpc_indirect_excitation_test.cpp
            mpc_synchronous_machine_infinite_bus_test.cpp test_parameters.h)

target_link_libraries(
    ${PROJECT_NAME} PRIVATE gtest::gtest_main
                            yeet::yeet_application::mpc_integration)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
