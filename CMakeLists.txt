cmake_minimum_required(VERSION 3.25)

# ASCII art banner because we're extra ✨
message(
    STATUS
        [[

        __    __  ____    ____    ______
        /\ \  /\ \/\  _`\ /\  _`\ /\__  _\
        \ `\`\\/'/\ \ \L\_\ \ \L\_\/_/\ \/
         `\ `\ /'  \ \  _\L\ \  _\L  \ \ \
           `\ \ \   \ \ \L\ \ \ \L\ \ \ \ \
             \ \_\   \ \____/\ \____/  \ \_\
              \/_/    \/___/  \/___/    \/_/
]])

project(
    yeet_project
    VERSION 1.0.0.0
    DESCRIPTION "No 🧢 Yeet Platform fr fr"
    HOMEPAGE_URL "https://github.com/marciomoura/yeet/"
    LANGUAGES C CXX)

# Define all options upfront with proper descriptions - keeping it 💯
option(YEET_CODE_COVERAGE
       "Enable code coverage (because we test our code like bosses)" OFF)
option(YEET_SANITIZE "Enable clang-sanitizer builds (catch those bugs bestie)"
       OFF)
option(YEET_MEMCHECK
       "Enable memcheck builds (memory leaks? not in this house 💅)" OFF)
option(YEET_BUILD_TESTING "Build the testing tree (we stay tested 💯)" ON)
option(YEET_BUILD_EXAMPLES "Build the examples tree (show me the receipts 👀)"
       ON)
option(YEET_ENABLE_CODE_ANALYSIS "Run code analysis (vibe check our code)" OFF)
option(YEET_ENABLE_CCACHE "Enable ccache (speed goes brrr 🏃‍♂️💨)" ON)
option(
    YEET_PRECISION_DOUBLE
    "Use double precision for floating-point types (because we like it precise 💎)"
    ON)
# Option to disable data logging for faster CI execution
option(YEET_ENABLE_DATA_LOGGING "Enable JSON data logging for simulations" ON)

# Set output directories before any targets are created (keeping it organized
# bestie)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Module path configuration - stack that bread 💸
list(
    APPEND
    CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependencies")

# Language standards configuration - no cap, we stay modern
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Propagate CMake option to a C++ preprocessor definition
if(YEET_PRECISION_DOUBLE)
    add_compile_definitions(YEET_PRECISION_DOUBLE)
    message(
        STATUS
            "yeet build-config: Double precision enabled (the only way to be ✨)"
    )
else()
    message(
        STATUS
            "yeet build-config: Single precision enabled (for the speed demons 💨)"
    )
endif()

if(YEET_ENABLE_DATA_LOGGING)
    add_compile_definitions(YEET_ENABLE_DATA_LOGGING)
endif()

# Generator-specific configuration - checking the fit
if(CMAKE_GENERATOR STREQUAL "Ninja")
    message(
        STATUS
            "yeet build-config: Using Ninja generator - go ninja go ninja go! 🥷"
    )
else()
    message(
        STATUS
            "yeet build-config: Using ${CMAKE_GENERATOR} generator - weird flex but ok 🤔"
    )
endif()

# Conditional includes based on options - let's get this bread 🍞
if(YEET_BUILD_TESTING)
    message(
        STATUS
            "yeet build-config: Build testing enabled - we stay testing fr fr ✅"
    )
    include(gtest)
    enable_testing()
else()
    message(
        STATUS
            "yeet build-config: Build testing disabled - living life on the edge bestie ⚠️"
    )
endif()

if(YEET_ENABLE_CCACHE)
    message(
        STATUS "yeet build-config: CCCache enabled - speedy builds go brrr 🏎️")
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
        set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    else()
        message(WARNING "CCCache not found - this ain't it chief 😔")
    endif()
else()
    message(
        STATUS "yeet build-config: CCCache disabled - chose violence today 😤")
endif()

if(YEET_ENABLE_CODE_ANALYSIS)
    message(
        STATUS
            "yeet build-config: Static code analysis enabled - checking the vibes 🔍"
    )
    include(cppcheck)
else()
    message(
        STATUS
            "yeet build-config: Static code analysis disabled - living dangerously bestie 👀"
    )
endif()

# Required packages - getting the squad together
message(
    STATUS
        "yeet build-config: Searching for JSON package - where you at bestie? 🔍"
)

include(json)

# Source directories - let's get this party started 🎉
add_subdirectory(software/yeet)
add_subdirectory(software/yeet_testing)
add_subdirectory(software/yeet_control)

# Build the SFC application
add_subdirectory(software/yeet_application/sfc)
add_subdirectory(software/yeet_application/sfc_integration)

if(YEET_BUILD_TESTING)
    add_subdirectory(software/yeet/tests)
    add_subdirectory(software/yeet_control/tests)
    add_subdirectory(software/yeet_testing/tests)
    add_subdirectory(software/yeet_application/sfc/tests)
    add_subdirectory(software/yeet_application/sfc_integration/tests)
endif()

if(YEET_CODE_COVERAGE)
    message(
        STATUS
            "yeet build-config: Add coverage target for generating code-coverage report"
    )

    find_program(GCOVR gcovr REQUIRED)
    # Create bin/coverage directory for storing coverage report
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/coverage)

    add_custom_target(
        coverage
        COMMAND
            ${GCOVR} --gcov-executable "llvm-cov gcov" --html --html-details
            --output "${CMAKE_BINARY_DIR}/coverage/index.html" --print-summary
            --exclude ".*build.*" --exclude ".*_deps.*" --exclude ".*tests.*"
            --gcov-ignore-parse-errors --fail-under-line 90
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating coverage report with gcovr...")
endif()

message(STATUS "Build config complete - let's get this bread! 🚀")
