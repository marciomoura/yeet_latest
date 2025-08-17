project(yeet_application_sfc_integration_tests CXX)

add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_application::sfc_integration_tests ALIAS
               ${PROJECT_NAME})

enable_testing()

target_sources(
    ${PROJECT_NAME}
    PRIVATE open_loop_plant_model_test.cpp
            sensorless_current_controller_test.cpp
            simplified_current_controller_test.cpp
            simplified_speed_controller_test.cpp)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE gtest::gtest_main
            yeet::yeet
            yeet::yeet_testing
            yeet::yeet_application_sfc
            yeet::yeet_application_sfc_integration)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
