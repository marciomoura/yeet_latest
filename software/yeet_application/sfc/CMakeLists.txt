project(yeet_application_sfc CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_application_sfc ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME} PUBLIC include/sfc/sfc_per_unit_system.hpp)

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC yeet::yeet_control)

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
