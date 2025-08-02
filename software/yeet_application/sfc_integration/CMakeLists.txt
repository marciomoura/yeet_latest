project(yeet_application_sfc_integration CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_application::sfc_integration ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PRIVATE include/sfc_integration/sfc_datamodel.hpp
            include/sfc_integration/sfc_grid_observer_runner.hpp
    PUBLIC src/sfc_grid_observer_runner.cpp)

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC yeet::yeet_application_sfc
                                             yeet::yeet yeet::yeet_testing)

# if(ENABLE_CODE_ANALYSIS) get_target_property(TARGET_SOURCES ${PROJECT_NAME}
# SOURCES) add_custom_target(${PROJECT_NAME}_cppcheck DEPENDS ${PROJECT_NAME}
# COMMAND ${CMAKE_CXX_CPPCHECK} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SOURCES}
# WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMENT "Static code analysis
# using cppcheck" ) endif()

# if(BUILD_TESTING) add_subdirectory(tests) endif()
