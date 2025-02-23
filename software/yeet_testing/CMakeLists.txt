project(yeet_testing CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_testing ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PUBLIC include/yeet_testing/parameter_collection.hpp
           include/yeet_testing/checker_failures_post_processing_interface.hpp
           include/yeet_testing/data_sample_json_post_processing.hpp
           include/yeet_testing/data_sample_post_processing_interface.hpp
           include/yeet_testing/data_sample_terminal_post_processing.hpp
           include/yeet_testing/data_sampler_entry.hpp
           include/yeet_testing/data_sampler.hpp
           include/yeet_testing/parameter_collection.hpp
           include/yeet_testing/simulator_checker_runner.hpp
           include/yeet_testing/data_sample_json_post_processing.hpp
           include/yeet_testing/data_sample_post_processing_interface.hpp
           include/yeet_testing/data_sample_terminal_post_processing.hpp
           include/yeet_testing/data_sampler_entry.hpp
           include/yeet_testing/data_sampler.hpp
           include/yeet_testing/simulator_checker.hpp
           include/yeet_testing/simulator_runner_application_interface.hpp
           include/yeet_testing/simulator_runner_application.hpp
           include/yeet_testing/simulator_runner.hpp
           include/yeet_testing/simulator_sampler_runner.hpp
           include/yeet_testing/simulator.hpp
           include/yeet_testing/testing_info_interface.hpp
    PRIVATE src/dummy.cpp)

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC nlohmann_json::nlohmann_json yeet::yeet
    PRIVATE gtest::gtest)

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
