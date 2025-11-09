project(yeet_application_sfc_tests CXX)

# Create the test executable
add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_application_sfc_tests ALIAS ${PROJECT_NAME})

enable_testing()

# Add test sources
target_sources(
    ${PROJECT_NAME}
    PRIVATE grid_side_hardware_mock_test.cpp
            machine_side_hardware_mock_test.cpp
            sfc_combined_modulators_test.cpp
            sfc_commutation_mask_test.cpp
            sfc_discontinuous_conduction_detector_test.cpp
            sfc_excitation_controller_test.cpp
            sfc_grid_side_measurements_test.cpp
            sfc_grid_side_modulator_test.cpp
            sfc_machine_side_lookup_table_test.cpp
            sfc_machine_side_measurements_test.cpp
            sfc_machine_side_modulator_test.cpp
            sfc_mode_detector_test.cpp
            sfc_per_unit_system_test.cpp
            sfc_soft_starter_speed_generator_test.cpp
            sfc_startup_sequencer_test.cpp
            sfc_state_machine_test.cpp
            sfc_torque_controller_test.cpp)

# Link dependencies
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE gtest::gtest_main
            gtest::gtest
            gtest::gmock
            yeet::yeet_application_sfc
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
    message(STATUS "Enabling coverage reporting with gcovr for SFC tests")
    # These flags tell the compiler to add instrumentation for coverage tracking
    target_compile_options(${PROJECT_NAME} PRIVATE -fprofile-arcs
                                                   -ftest-coverage)
    target_link_options(${PROJECT_NAME} PRIVATE --coverage)
endif()
