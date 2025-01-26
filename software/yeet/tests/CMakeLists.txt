project(yeet_test CXX)

add_executable(${PROJECT_NAME})
add_executable(yeet::tests ALIAS ${PROJECT_NAME})

enable_testing()

target_sources(${PROJECT_NAME} PRIVATE
    parameter_test.cpp
    signal_test.cpp
    utils/low_level_access_test.cpp
    examples/control_example_test.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE gtest::gtest_main gtest::gtest gtest::gmock yeet::yeet)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
