project(yeet_control CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_control ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PUBLIC
        include/yeet_control/angle.hpp
        include/yeet_control/assynchronous_machine_per_unit.hpp
        include/yeet_control/assynchronous_machine.hpp
        include/yeet_control/asynchronous_machine_vector_control.hpp
        include/yeet_control/constants.hpp
        include/yeet_control/externally_excited_synchronous_machine_per_unit.hpp
        include/yeet_control/externally_excited_synchronous_machine.hpp
        include/yeet_control/first_order_low_pass_filter.hpp
        include/yeet_control/integrator.hpp
        include/yeet_control/limit.hpp
        include/yeet_control/machine_current_controller.hpp
        include/yeet_control/machine_flux_controller.hpp
        include/yeet_control/machine_measurements.hpp
        include/yeet_control/machine_per_unit_system.hpp
        include/yeet_control/machine_speed_controller.hpp
        include/yeet_control/mechanical_load.hpp
        include/yeet_control/per_unit.hpp
        include/yeet_control/permanent_magnet_synchronous_machine_per_unit.hpp
        include/yeet_control/permanent_magnet_synchronous_machine.hpp
        include/yeet_control/pi_controller.hpp
        include/yeet_control/precomputed_divisor.hpp
        include/yeet_control/pulse_generator.hpp
        include/yeet_control/range_kutta_derivative.hpp
        include/yeet_control/reference_frame.hpp
        include/yeet_control/second_order_filter.hpp
        include/yeet_control/six_pulse_generator.hpp
        include/yeet_control/three_phase_waveform_generator.hpp
        include/yeet_control/type_traits.hpp
        include/yeet_control/types.hpp
        include/yeet_control/units.hpp
    PRIVATE src/angle.cpp
            src/mechanical_load.cpp
            src/pi_controller.cpp
            src/pulse_generator.cpp
            src/six_pulse_generator.cpp)

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
