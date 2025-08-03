project(yeet_control CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_control ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PUBLIC include/yeet_control/adaptive_band_reject_filter.hpp
           include/yeet_control/angle_wrapped.hpp
           include/yeet_control/constants.hpp
           include/yeet_control/derivative.hpp
           include/yeet_control/dual_sogi_srf_pll.hpp
           include/yeet_control/edge_detector.hpp
           include/yeet_control/elapsed_timer.hpp
           include/yeet_control/first_order_low_pass_filter.hpp
           include/yeet_control/integrator.hpp
           include/yeet_control/interval_timer.hpp
           include/yeet_control/limit.hpp
           include/yeet_control/lookup_table_2d.hpp
           include/yeet_control/mechanical_load.hpp
           include/yeet_control/off_delay.hpp
           include/yeet_control/on_delay.hpp
           include/yeet_control/per_unit.hpp
           include/yeet_control/pi_controller.hpp
           include/yeet_control/precomputed_divisor.hpp
           include/yeet_control/pulse_generator.hpp
           include/yeet_control/rate_of_change_limiter.hpp
           include/yeet_control/range_kutta_derivative.hpp
           include/yeet_control/reference_frame.hpp
           include/yeet_control/second_order_filter.hpp
           include/yeet_control/six_pulse_generator.hpp
           include/yeet_control/sogi_filter_sequence_extractor.hpp
           include/yeet_control/sogi_filter.hpp
           include/yeet_control/three_phase_pll.hpp
           include/yeet_control/three_phase_waveform_generator.hpp
           include/yeet_control/type_traits.hpp
           include/yeet_control/types.hpp
           include/yeet_control/units.hpp
           include/yeet_control/speed_voltage_relationship.hpp
    PRIVATE src/derivative.cpp
            src/dual_sogi_srf_pll.cpp
            src/edge_detector.cpp
            src/elapsed_timer.cpp
            src/mechanical_load.cpp
            src/interval_timer.cpp
            src/off_delay.cpp
            src/on_delay.cpp
            src/pi_controller.cpp
            src/pulse_generator.cpp
            src/rate_of_change_limiter.cpp
            src/six_pulse_generator.cpp
            src/sogi_filter_sequence_extractor.cpp
            src/sogi_filter.cpp
            src/three_phase_pll.cpp
            src/speed_voltage_relationship.cpp)

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
