project(yeet_control_tests CXX)

# Create the test executable
add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_control_tests ALIAS ${PROJECT_NAME})

enable_testing()

# Add test sources
target_sources(
    ${PROJECT_NAME}
    PRIVATE angle_test.cpp
            assynchronous_machine_test.cpp
            assynchronous_machine_per_unit_test.cpp
            asynchronous_machine_vector_control_test.cpp
            externally_excited_synchronous_machine_per_unit_test.cpp
            externally_excited_synchronous_machine_test.cpp
            externally_excited_synchronous_machine_vector_control_test.cpp
            first_order_low_pass_filter_test.cpp
            integrator_test.cpp
            limit_tests.cpp
            mechanical_load_test.cpp
            per_unit_test.cpp
            permanent_magnet_synchronous_machine_per_unit_test.cpp
            permanent_magnet_synchronous_machine_test.cpp
            permanent_magnet_synchronous_machine_vector_control_test.cpp
            pi_controller_test.cpp
            pulse_generator_test.cpp
            pulse_generator_plot_test.cpp
            reference_frame_test.cpp
            second_order_filter_test.cpp
            six_pulse_generator_test.cpp
            six_pulse_generator_plot_test.cpp
            three_phase_waveform_generator_test.cpp)

# Link dependencies
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE gtest::gtest_main
            gtest::gtest
            gtest::gmock
            yeet::yeet_control
            yeet::yeet_testing
            $<$<NOT:$<PLATFORM_ID:Windows>>:pthread>)

# Discover tests
include(GoogleTest)
gtest_discover_tests(
    ${PROJECT_NAME}
    PROPERTIES LABELS "unit" # Add labels for better organization
    TIMEOUT 30 # Add timeout for each test
)

if(YEET_CODE_COVERAGE)
    message(STATUS "Enabling coverage reporting with gcovr")
    # These flags tell the compiler to add instrumentation for coverage tracking
    target_compile_options(${PROJECT_NAME} PRIVATE -fprofile-arcs
                                                   -ftest-coverage)
    target_link_options(${PROJECT_NAME} PRIVATE --coverage)
endif()
