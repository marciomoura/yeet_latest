# Excitation MPC
# Description: MPC static target

project(yeet CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PUBLIC include/yeet/application_datamodel.hpp
           include/yeet/application_interface.hpp
           include/yeet/application_registry.hpp
           include/yeet/control_execution_policy.hpp
           include/yeet/control_task_level_registry.hpp
           include/yeet/control_task_level.hpp
           include/yeet/data_model_connector.hpp
           include/yeet/model_registry.hpp
           include/yeet/parameter.hpp
           include/yeet/signal.hpp
           include/yeet/utils/low_level_access.hpp
           include/yeet/utils/low_level_access_registry.hpp
           include/yeet/utils/metadata.hpp
           include/yeet/utils/storage_type.hpp
    PRIVATE src/parameter.cpp)

target_include_directories(
    ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/examples)

target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)

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
