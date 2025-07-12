project(yeet_application_sfc CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_application_sfc ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PUBLIC
        include/sfc/sfc_per_unit_system.hpp
    PRIVATE
        # src/sfc_measurement.cpp
)

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC yeet::yeet_control)

if(YEET_ENABLE_TESTING)
    add_executable(yeet_application_sfc_tests
        tests/pulse_generator_test.cpp
        tests/six_pulse_generator_test.cpp
    )
    target_link_libraries(yeet_application_sfc_tests
        PRIVATE
            GTest::gtest_main
            yeet::yeet_application_sfc
            yeet::yeet_testing
    )
    add_test(NAME ${PROJECT_NAME}_tests COMMAND yeet_application_sfc_tests)
endif()

if(YEET_ENABLE_CODE_ANALYSIS)
    get_target_property(TARGET_SOURCES ${PROJECT_NAME} SOURCES)
    add_custom_target(
        ${PROJECT_NAME}_cppcheck
        DEPENDS ${PROJECT_NAME}
        COMMAND ${CMAKE_CXX_CPPCHECK}
                ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SOURCES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Static code analysis using cppcheck")
endif()
