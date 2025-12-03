project(yeet_application_sfc_integration_tests CXX)

add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_application::sfc_integration_tests ALIAS
               ${PROJECT_NAME})

enable_testing()

target_sources(
    ${PROJECT_NAME}
    PRIVATE # Helpers
            sfc_loggers.hpp
            sfc_plant_configuration.hpp
            # Complete closed loop tests
            complete_sensorless_speed_control_test.cpp
            closed_loop_rectifier_test.cpp
            # Open loop tests
            open_loop_back_to_back_test.cpp
            open_loop_rectifier_test.cpp
            # Current control tests
            current_control_forced_commutation_test.cpp
            current_control_natural_commutation_test.cpp
            current_control_rectifier_test.cpp
            # Speed control tests
            speed_control_test.cpp
            # State machine tests
            sfc_state_machine_runner_test.cpp
            # Excitation controller tests
            sfc_excitation_controller_runner_test.cpp
            # Protection tests
            sfc_grid_side_protections_alarm_confirmation_test.cpp
            sfc_machine_side_protections_alarm_confirmation_test.cpp
            protection_alarm_confirmation_integration_test.cpp)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE gtest::gtest_main
            yeet::yeet
            yeet::yeet_testing
            yeet::yeet_application_sfc
            yeet::yeet_application_sfc_integration)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
