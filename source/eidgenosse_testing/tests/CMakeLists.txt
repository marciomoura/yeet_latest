project(eidgenosse_testing_tests CXX)

add_executable(${PROJECT_NAME})
add_executable(eidgenosse::eidgenosse_testing_tests ALIAS ${PROJECT_NAME})

enable_testing()

target_sources(${PROJECT_NAME} PRIVATE testing_test.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE gtest::gtest_main eidgenosse::eidgenosse eidgenosse::eidgenosse_testing)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
