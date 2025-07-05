project(yeet_generated_indirect_excitation_2mva_plant_model CXX)

add_library(${PROJECT_NAME} STATIC)

target_sources(
    ${PROJECT_NAME}
    PUBLIC indirect_excitation_2mva_plant_model.h
    PRIVATE indirect_excitation_2mva_plant_model_private.h
            indirect_excitation_2mva_plant_model_types.h
            rt_defines.h
            rt_look.h
            rt_look1d.h
            rt_nonfinite.h
            rtGetInf.h
            rtGetNaN.h
            rtwtypes.h
            indirect_excitation_2mva_plant_model_data.cpp
            indirect_excitation_2mva_plant_model.cpp
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
