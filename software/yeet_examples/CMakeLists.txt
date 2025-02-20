project(yeet_examples CXX)

# Create the test executable
add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_examples ALIAS ${PROJECT_NAME})

enable_testing()

# Add test sources
target_sources(${PROJECT_NAME} PRIVATE control_integration.hpp
                                       control_example_test.cpp)

# Link dependencies
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE gtest::gtest_main
            gtest::gtest
            gtest::gmock
            yeet::yeet
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
