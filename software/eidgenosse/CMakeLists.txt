# Excitation MPC
# Description: MPC static target

project(eidgenosse CXX)

add_library(${PROJECT_NAME} STATIC)
add_library(
  eidgenosse::eidgenosse
  ALIAS
  ${PROJECT_NAME})

target_sources(
  ${PROJECT_NAME}
  PUBLIC
  include/eidgenosse/application_interface.hpp
  include/eidgenosse/application_registry.hpp
  include/eidgenosse/control_execution_policy.hpp
  include/eidgenosse/control_task_level_registry.hpp
  include/eidgenosse/control_task_level.hpp
  include/eidgenosse/model_registry.hpp
  include/eidgenosse/parameter.hpp
  include/eidgenosse/signal.hpp
  include/eidgenosse/utils/low_level_access.hpp
  include/eidgenosse/utils/low_level_access_registry.hpp
  include/eidgenosse/utils/metadata.hpp
  include/eidgenosse/utils/storage_type.hpp
  include/eidgenosse/testing/control_task_simulator_runner.hpp
  include/eidgenosse/testing/data_sample_json_post_processing.hpp
  include/eidgenosse/testing/data_sample_post_processing_interface.hpp
  include/eidgenosse/testing/data_sample_terminal_post_processing.hpp
  include/eidgenosse/testing/data_sampler_entry.hpp
  include/eidgenosse/testing/data_sampler.hpp
  include/eidgenosse/testing/simulator_runner_application_interface.hpp
  include/eidgenosse/testing/simulator.hpp
  include/eidgenosse/testing/testing_info_interface.hpp
  PRIVATE src/parameter.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                  ${CMAKE_CURRENT_SOURCE_DIR}/examples)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  nlohmann_json::nlohmann_json)

if(EIDGENOSSE_ENABLE_CODE_ANALYSIS)
  get_target_property(
    TARGET_SOURCES
    ${PROJECT_NAME}
    SOURCES)
  add_custom_target(
    ${PROJECT_NAME}_cppcheck
    DEPENDS ${PROJECT_NAME}
    COMMAND ${CMAKE_CXX_CPPCHECK} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Static code analysis using cppcheck")
endif()

if(EIDGENOSSE_BUILD_TESTING)
  add_subdirectory(examples)
  add_subdirectory(tests)
endif()
