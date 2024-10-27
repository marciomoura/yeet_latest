project(eidgenosse_test CXX)

add_executable(${PROJECT_NAME})
add_executable(eidgenosse::tests ALIAS ${PROJECT_NAME})

enable_testing()

target_sources(${PROJECT_NAME} PRIVATE 
    parameter_test.cpp
    signal_test.cpp
    utils/low_level_access_test.cpp 
) 

target_link_libraries(${PROJECT_NAME} PRIVATE gtest::gtest_main eidgenosse::eidgenosse)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
