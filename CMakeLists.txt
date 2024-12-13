cmake_minimum_required(VERSION 3.25)

project(
  eidgenosse_platform
  VERSION 1.0.0.0
  DESCRIPTION "Eidgenosse Platform"
  HOMEPAGE_URL "https://github.com/marciomoura/eidgenosse_platform/"
  LANGUAGES CXX)

# C++ Language settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Append .cmake files to the list of available files
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependencies)

# Project configuration options
if(NOT DEFINED EIDGENOSSE_CODE_COVERAGE)
  option(EIDGENOSSE_CODE_COVERAGE "Enable code coverage" OFF)
endif()

if(NOT DEFINED EIDGENOSSE_SANITIZE)
  option(EIDGENOSSE_SANITIZE "Enable clang-sanitizer instrumented builds" OFF)
endif()

if(NOT DEFINED EIDGENOSSE_MEMCHECK)
  option(EIDGENOSSE_MEMCHECK "Enable memcheck instrumented builds" OFF)
endif()

if(NOT DEFINED EIDGENOSSE_STATIC_LINKAGE)
  option(EIDGENOSSE_STATIC_LINKAGE "Compile with static linkage" OFF)
endif()

if(NOT DEFINED EIDGENOSSE_BUILD_TESTING)
  option(EIDGENOSSE_BUILD_TESTING "Build the testing tree." OFF)
endif()

if(NOT DEFINED EIDGENOSSE_ENABLE_CODE_ANALYSIS)
  option(EIDGENOSSE_ENABLE_CODE_ANALYSIS "Run code analysis" OFF)
endif()

if(NOT DEFINED EIDGENOSSE_ENABLE_CCACHE)
  option(EIDGENOSSE_ENABLE_CCACHE "Enable ccache" ON)
endif()

# Check if Ninja is being used as the generator
if (CMAKE_GENERATOR STREQUAL "Ninja")
  message(STATUS "-- build-config: ✅ using Ninja generator")
else ()
  message(STATUS "-- build-config: ⬜ using ${CMAKE_GENERATOR} generator")
endif()

# Include test components
if(EIDGENOSSE_BUILD_TESTING)
  message(STATUS "-- build-config: ✅ enabled build testing")
  include(gtest)
else()
  message(STATUS "-- build-config: ⬜ build testing not enabled")
endif()

# Include ccache
if(EIDGENOSSE_ENABLE_CCACHE)
  message(STATUS "-- build-config: ✅ enabled ccache")
  include(ccache)
else()
  message(STATUS "-- build-config: ⬜ ccache not enabled")
endif()

# Include code analysis components
if(EIDGENOSSE_ENABLE_CODE_ANALYSIS)
  message(STATUS "-- build-config: ✅ enabled static code analysis")
  include(cppcheck)
else()
  message(STATUS "-- build-config: ⬜ static code analysis not enabled")
endif()

# Include code analysis components
message(STATUS "-- build-config: 🟢 searching json package")
include(json)

# Add source-code directories
add_subdirectory(software/eidgenosse)
