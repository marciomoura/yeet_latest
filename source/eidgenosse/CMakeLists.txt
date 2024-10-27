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
  PUBLIC include/eidgenosse/application_interface.hpp
         include/eidgenosse/model_registry.hpp
         include/eidgenosse/parameter.hpp
         include/eidgenosse/signal.hpp
         include/eidgenosse/utils/low_level_access.hpp
         include/eidgenosse/utils/low_level_access_registry.hpp
         include/eidgenosse/utils/metadata.hpp
         include/eidgenosse/utils/storage_type.hpp
  PRIVATE src/parameter.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC)

if(ENABLE_CODE_ANALYSIS)
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

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
