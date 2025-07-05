project(yeet_generated_synchronous_machine_infinite_bus CXX)

add_library(${PROJECT_NAME} STATIC)

target_sources(
    ${PROJECT_NAME}
    PRIVATE plant_schematic_private.h
            plant_schematic_types.h
            plant_schematic.h
            rt_defines.h
            rt_look.h
            rt_look1d.h
            rt_nonfinite.h
            rtGetInf.h
            rtGetNaN.h
            rtwtypes.h
    PUBLIC plant_schematic_data.cpp
           plant_schematic.cpp
           rt_look.cpp
           rt_look1d.cpp
           rt_nonfinite.cpp
           rtGetInf.cpp
           rtGetNaN.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC)

# if(ENABLE_CODE_ANALYSIS) get_target_property(TARGET_SOURCES ${PROJECT_NAME}
# SOURCES) add_custom_target(${PROJECT_NAME}_cppcheck DEPENDS ${PROJECT_NAME}
# COMMAND ${CMAKE_CXX_CPPCHECK} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SOURCES}
# WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMENT "Static code analysis
# using cppcheck" ) endif()

# if(BUILD_TESTING) add_subdirectory(tests) endif()
