project(yeet_application_sfc_integration__tests CXX)

add_executable(${PROJECT_NAME})
add_executable(yeet::yeet_application::sfc_integration__tests ALIAS
               ${PROJECT_NAME})

enable_testing()

target_sources(${PROJECT_NAME} PRIVATE dummy.cpp)

target_link_libraries(
    ${PROJECT_NAME} PRIVATE gtest::gtest_main
                            yeet::yeet_application::sfc_integration)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
