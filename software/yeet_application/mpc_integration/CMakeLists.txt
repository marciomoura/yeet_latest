project(yeet_application_mpc_integration CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_application::mpc_integration ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PRIVATE
        include/mpc_integration/indirect_excitation_2mva_plant_model_integration_runner.hpp
        include/mpc_integration/mpc_integration_runner.hpp
        include/mpc_integration/mpc_integration_model.hpp
        include/mpc_integration/synchronous_machine_infinite_bus_integration_model.hpp
        include/mpc_integration/synchronous_machine_infinite_bus_integration_runner.hpp
        include/mpc_integration/testing_closed_loop_mpc.hpp
    PUBLIC src/indirect_excitation_2mva_plant_model_integration_runner.cpp
           src/mpc_integration_model.cpp
           src/mpc_integration_runner.cpp
           src/synchronous_machine_infinite_bus_integration_runner.cpp)

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC yeet::yeet_application::mpc
           yeet::yeet
           yeet::yeet_testing
           yeet_generated_synchronous_machine_infinite_bus
           yeet_generated_indirect_excitation_2mva_plant_model)

# if(ENABLE_CODE_ANALYSIS) get_target_property(TARGET_SOURCES ${PROJECT_NAME}
# SOURCES) add_custom_target(${PROJECT_NAME}_cppcheck DEPENDS ${PROJECT_NAME}
# COMMAND ${CMAKE_CXX_CPPCHECK} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SOURCES}
# WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMENT "Static code analysis
# using cppcheck" ) endif()

# if(BUILD_TESTING) add_subdirectory(tests) endif()
