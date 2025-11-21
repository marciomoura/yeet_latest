project(yeet_control_tests CXX)

# Create the test executable
add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_control_tests ALIAS ${PROJECT_NAME})

enable_testing()

# Add test sources
target_sources(
    ${PROJECT_NAME}
    PRIVATE adc_value_test.cpp
            adc_mock_test.cpp
            angle_test.cpp
            derivative_test.cpp
            dual_sogi_srf_pll_test.cpp
            edge_detector_test.cpp
            elapsed_timer_test.cpp
            first_order_low_pass_filter_test.cpp
            hysteresis_limiter_test.cpp
            integrator_test.cpp
            interval_timer_test.cpp
            limit_tests.cpp
            linear_ramp_test.cpp
            lookup_table_1d_test.cpp
            lookup_table_2d_test.cpp
            mechanical_load_test.cpp
            off_delay_test.cpp
            on_delay_test.cpp
            on_off_delay_test.cpp
            per_unit_test.cpp
            pi_controller_test.cpp
            precomputed_divisor_test.cpp
            pulse_generator_test.cpp
            rate_of_change_limiter_test.cpp
            pulse_generator_plot_test.cpp
            reference_frame_test.cpp
            second_order_filter_test.cpp
            six_pulse_generator_test.cpp
            six_pulse_generator_plot_test.cpp
            sogi_filter_sequence_extractor_test.cpp
            sogi_filter_test.cpp
            srf_pll_test.cpp
            threshold_protection_test.cpp
            three_phase_pll_test.cpp
            # three_phase_waveform_generator_test.cpp - Temporarily disabled due
            # to test issues
            thyristor_bridge_test.cpp
            thyristor_rectifier_test.cpp
            thyristor_inverter_test.cpp
            # thyristor_back_to_back_test.cpp
            # thyristor_back_to_back_synchronous_machine_test.cpp
            synchronous_machine_backemf_test.cpp
            units_test.cpp
            speed_voltage_relationship_test.cpp
            virtual_angle_reference_generator_test.cpp
            synchronous_machine_test.cpp)

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
