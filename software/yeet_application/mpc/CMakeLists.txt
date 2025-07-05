project(yeet_application_mpc CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_application::mpc ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PRIVATE include/mpc/mpc.h
            include/mpc/model.h
            include/mpc/incremental_format.h
            include/mpc/quadratic_program.h
            include/mpc/qp_solver.h
    PUBLIC src/model.cpp)

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen qpOASES
                                             yeet::yeet_control)

target_compile_options(
    ${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-error>
                            $<$<CONFIG:Release>:-O3>)

if(ENABLE_CODE_ANALYSIS)
    get_target_property(TARGET_SOURCES ${PROJECT_NAME} SOURCES)
    add_custom_target(
        ${PROJECT_NAME}_cppcheck
        DEPENDS ${PROJECT_NAME}
        COMMAND ${CMAKE_CXX_CPPCHECK}
                ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SOURCES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Static code analysis using cppcheck")
endif()
