# project(yeet_control CXX)

# add_library(${PROJECT_NAME} STATIC) add_library(yeet::yeet_control ALIAS
# ${PROJECT_NAME})

# target_sources( ${PROJECT_NAME} PUBLIC PRIVATE)

# target_include_directories( ${PROJECT_NAME} PUBLIC
# ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/examples)

# target_link_libraries(${PROJECT_NAME} PUBLIC)

# if(YEET_ENABLE_CODE_ANALYSIS) get_target_property(TARGET_SOURCES
# ${PROJECT_NAME} SOURCES) add_custom_target( ${PROJECT_NAME}_cppcheck DEPENDS
# ${PROJECT_NAME} COMMAND ${CMAKE_CXX_CPPCHECK}
# ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SOURCES} WORKING_DIRECTORY
# ${CMAKE_CURRENT_SOURCE_DIR} COMMENT "Static code analysis using cppcheck")
# endif()
