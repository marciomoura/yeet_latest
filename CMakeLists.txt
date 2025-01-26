cmake_minimum_required(VERSION 3.25)

project(
  yeet_project
  VERSION 1.0.0.0
  DESCRIPTION "Yeet Platform"
  HOMEPAGE_URL "https://github.com/marciomoura/yeet/"
  LANGUAGES C CXX)

# Define all options upfront with proper descriptions
option(YEET_CODE_COVERAGE "Enable code coverage" OFF)
option(YEET_SANITIZE "Enable clang-sanitizer instrumented builds" OFF)
option(YEET_MEMCHECK "Enable memcheck instrumented builds" OFF)
option(YEET_STATIC_LINKAGE "Compile with static linkage" OFF)
option(YEET_BUILD_TESTING "Build the testing tree" ON)
option(YEET_BUILD_EXAMPLES "Build the examples tree" ON)
option(YEET_ENABLE_CODE_ANALYSIS "Run code analysis" OFF)
option(YEET_ENABLE_CCACHE "Enable ccache" ON)

# Set output directories before any targets are created
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Module path configuration
list(APPEND CMAKE_MODULE_PATH 
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependencies"
)

# Language standards configuration
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Generator-specific configuration
if(CMAKE_GENERATOR STREQUAL "Ninja")
    message(STATUS "yeet build-config: Using Ninja generator ✅")
else()
    message(STATUS "yeet build-config: Using ${CMAKE_GENERATOR} generator ⬜")
endif()

# Conditional includes based on options
if(YEET_BUILD_TESTING)
    message(STATUS "yeet build-config: Build testing enabled ✅")
    include(gtest)
    enable_testing()  # Added to properly enable testing
else()
    message(STATUS "yeet build-config: Build testing disabled ⬜")
endif()

if(YEET_ENABLE_CCACHE)
    message(STATUS "yeet build-config: CCCache enabled ✅")
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
        set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    else()
        message(WARNING "CCCache not found, continuing without it")
    endif()
else()
    message(STATUS "yeet build-config: CCCache disabled ⬜")
endif()

if(YEET_ENABLE_CODE_ANALYSIS)
    message(STATUS "yeet build-config: Static code analysis enabled ✅")
    include(cppcheck)
else()
    message(STATUS "yeet build-config: Static code analysis disabled ⬜")
endif()

# Required packages
message(STATUS "yeet build-config: Searching for JSON package 🔍")
include(json)

# Source directories
add_subdirectory(software/yeet)
if(YEET_BUILD_TESTING)
    add_subdirectory(software/yeet/tests)
endif()

# if(YEET_BUILD_EXAMPLES)
    # add_subdirectory(software/yeet/examples)
# endif()