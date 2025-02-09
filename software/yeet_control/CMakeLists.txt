project(yeet_control CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_control ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PUBLIC include/yeet_control/angle.hpp
           include/yeet_control/constants.hpp
           include/yeet_control/limit.hpp
           include/yeet_control/reference_frame.hpp
           include/yeet_control/first_order_low_pass_filter.hpp
           include/yeet_control/second_order_filter.hpp
           include/yeet_control/integrator.hpp
           include/yeet_control/pi_controller.hpp
    PRIVATE src/angle.cpp)

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC)

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
