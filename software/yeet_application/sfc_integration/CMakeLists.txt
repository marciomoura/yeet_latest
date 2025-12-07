project(yeet_application_sfc_integration CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_application_sfc_integration ALIAS ${PROJECT_NAME})

# Separate plant simulation builders (only needed for testing)
set(PLANT_SIMULATION_HEADERS
    include/sfc_integration/plant_hardware_definition.hpp
    include/sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp
    include/sfc_integration/plant_current_control_with_modulators_simulation_builder.hpp
    include/sfc_integration/plant_open_loop_simulation_builder.hpp
    include/sfc_integration/plant_sensorless_current_control_mocked_hardware_simulation_builder.hpp
    include/sfc_integration/plant_sensorless_current_control_simulation_builder.hpp
    include/sfc_integration/plant_sensorless_speed_control_mocked_hardware_simulation_builder.hpp
    include/sfc_integration/plant_sensorless_speed_control_simulation_builder.hpp
    include/sfc_integration/plant_simplified_current_control_mocked_hardware_simulation_builder.hpp
    include/sfc_integration/plant_simplified_current_control_simulation_builder.hpp
    include/sfc_integration/plant_speed_control_simulation_builder.hpp)

# Core SFC integration headers (always needed)
set(SFC_CORE_HEADERS
    include/sfc_integration/sfc_current_controller_datamodel.hpp
    include/sfc_integration/sfc_current_controller_runner.hpp
    include/sfc_integration/sfc_grid_side_external_ios_datamodel.hpp
    include/sfc_integration/sfc_machine_side_external_ios_datamodel.hpp
    include/sfc_integration/sfc_dc_link_measurements_datamodel.hpp
    include/sfc_integration/sfc_dc_link_measurements_runner.hpp
    include/sfc_integration/sfc_excitation_controller_datamodel.hpp
    include/sfc_integration/sfc_excitation_controller_runner.hpp
    include/sfc_integration/sfc_grid_side_control.hpp
    include/sfc_integration/sfc_grid_side_hardware_mock_datamodel.hpp
    include/sfc_integration/sfc_grid_side_hardware_mock_runner.hpp
    include/sfc_integration/sfc_io_mock_runner.hpp
    include/sfc_integration/sfc_grid_side_measurements_datamodel.hpp
    include/sfc_integration/sfc_grid_side_measurements_runner.hpp
    include/sfc_integration/sfc_grid_side_modulator_datamodel.hpp
    include/sfc_integration/sfc_grid_side_modulator_runner.hpp
    include/sfc_integration/sfc_grid_side_observer_runner.hpp
    include/sfc_integration/sfc_grid_side_observer_datamodel.hpp
    include/sfc_integration/sfc_grid_side_pulse_generator_datamodel.hpp
    include/sfc_integration/sfc_grid_side_pulse_generator_runner.hpp
    include/sfc_integration/sfc_grid_side_integrator_supervisor_datamodel.hpp
    include/sfc_integration/sfc_grid_side_integrator_supervisor_runner.hpp
    include/sfc_integration/sfc_grid_side_protections_datamodel.hpp
    include/sfc_integration/sfc_grid_side_protections_runner.hpp
    include/sfc_integration/sfc_machine_side_control.hpp
    include/sfc_integration/sfc_machine_side_controller_datamodel.hpp
    include/sfc_integration/sfc_machine_side_controller_runner.hpp
    include/sfc_integration/sfc_machine_side_hardware_mock_datamodel.hpp
    include/sfc_integration/sfc_machine_side_hardware_mock_runner.hpp
    include/sfc_integration/sfc_machine_side_measurements_datamodel.hpp
    include/sfc_integration/sfc_machine_side_measurements_runner.hpp
    include/sfc_integration/sfc_machine_side_modulator_datamodel.hpp
    include/sfc_integration/sfc_machine_side_modulator_runner.hpp
    include/sfc_integration/sfc_machine_side_observer_datamodel.hpp
    include/sfc_integration/sfc_machine_side_observer_runner.hpp
    include/sfc_integration/sfc_machine_side_pulse_generator_datamodel.hpp
    include/sfc_integration/sfc_machine_side_pulse_generator_runner.hpp
    include/sfc_integration/sfc_machine_side_integrator_supervisor_datamodel.hpp
    include/sfc_integration/sfc_machine_side_integrator_supervisor_runner.hpp
    include/sfc_integration/sfc_machine_side_protections_datamodel.hpp
    include/sfc_integration/sfc_machine_side_protections_runner.hpp
    include/sfc_integration/sfc_operation_mode_monitor_datamodel.hpp
    include/sfc_integration/sfc_operation_mode_monitor_runner.hpp
    include/sfc_integration/sfc_per_unit_system_datamodel.hpp
    include/sfc_integration/sfc_plant_model_datamodel.hpp
    include/sfc_integration/sfc_plant_model_runner.hpp
    include/sfc_integration/sfc_soft_starter_speed_generator_datamodel.hpp
    include/sfc_integration/sfc_soft_starter_speed_generator_runner.hpp
    include/sfc_integration/sfc_speed_controller_datamodel.hpp
    include/sfc_integration/sfc_speed_controller_runner.hpp
    include/sfc_integration/sfc_state_machine_datamodel.hpp
    include/sfc_integration/sfc_state_machine_runner.hpp
    include/sfc_integration/sfc_startup_sequencer_datamodel.hpp
    include/sfc_integration/sfc_startup_sequencer_runner.hpp
    include/sfc_integration/sfc_torque_controller_datamodel.hpp
    include/sfc_integration/sfc_torque_controller_runner.hpp)

# Core implementation sources (always needed)
set(SFC_CORE_SOURCES
    src/sfc_current_controller_runner.cpp
    src/sfc_dc_link_measurements_runner.cpp
    src/sfc_excitation_controller_runner.cpp
    src/sfc_grid_side_control.cpp
    src/sfc_grid_side_external_ios_datamodel.cpp
    src/sfc_grid_side_hardware_mock_runner.cpp
    src/sfc_io_mock_runner.cpp
    src/sfc_grid_side_measurements_runner.cpp
    src/sfc_grid_side_modulator_runner.cpp
    src/sfc_grid_side_observer_runner.cpp
    src/sfc_grid_side_protections_runner.cpp
    src/sfc_grid_side_pulse_generator_runner.cpp
    src/sfc_grid_side_integrator_supervisor_runner.cpp
    src/sfc_machine_side_control.cpp
    src/sfc_machine_side_controller_runner.cpp
    src/sfc_machine_side_external_ios_datamodel.cpp
    src/sfc_machine_side_hardware_mock_runner.cpp
    src/sfc_machine_side_measurements_runner.cpp
    src/sfc_machine_side_modulator_runner.cpp
    src/sfc_machine_side_observer_runner.cpp
    src/sfc_machine_side_pulse_generator_runner.cpp
    src/sfc_machine_side_integrator_supervisor_runner.cpp
    src/sfc_machine_side_protections_runner.cpp
    src/sfc_operation_mode_monitor_runner.cpp
    src/sfc_per_unit_system_datamodel.cpp
    src/sfc_plant_model_runner.cpp
    src/sfc_soft_starter_speed_generator_runner.cpp
    src/sfc_speed_controller_runner.cpp
    src/sfc_state_machine_runner.cpp
    src/sfc_startup_sequencer_runner.cpp
    src/sfc_torque_controller_runner.cpp)

# Add core sources (always included) Note: Sources must be PRIVATE to avoid
# double-compilation when linking executables
target_sources(${PROJECT_NAME} PRIVATE ${SFC_CORE_HEADERS} ${SFC_CORE_SOURCES})

# Only include plant simulation builders when building tests
if(YEET_BUILD_TESTING)
    message(STATUS "Including plant simulation builders for testing")
    target_sources(${PROJECT_NAME} PRIVATE ${PLANT_SIMULATION_HEADERS})
    target_compile_definitions(${PROJECT_NAME}
                               PUBLIC YEET_INCLUDE_PLANT_SIMULATIONS)
else()
    message(STATUS "Excluding plant simulation builders (not building tests)")
endif()

target_include_directories(${PROJECT_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC yeet::yeet_application_sfc
                                             yeet::yeet)

# Link testing libraries only when building tests
if(YEET_BUILD_TESTING)
    target_link_libraries(
        ${PROJECT_NAME}
        PRIVATE yeet::yeet_testing
                yeet::yeet_generated_sfc_6p_6p_topology # Auto-generated by
                                                        # Matlab
    )
endif()
