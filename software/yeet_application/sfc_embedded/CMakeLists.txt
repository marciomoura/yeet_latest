cmake_minimum_required(VERSION 3.22)

project(yeet_sfc_embedded)

# Create the embedded SFC application library
add_library(yeet_sfc_embedded STATIC)

# Add the task implementations
target_sources(yeet_sfc_embedded PRIVATE src/embedded_tasks.cpp)

# Public headers for the library
target_include_directories(
    yeet_sfc_embedded
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

# Link with SFC integration libraries (your control logic)
target_link_libraries(
    yeet_sfc_embedded
    PUBLIC yeet::yeet_control yeet::yeet_application_sfc_integration
    # yeet::yeet_application_sfc yeet::yeet_sfc_integration Note: These
    # libraries need to be embedded-safe (no JSON logging, no exceptions, etc.)
)

# For ARM embedded: Ensure proper compilation flags
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    target_compile_options(
        yeet_sfc_embedded
        PRIVATE -mthumb # Use Thumb instruction set
                -fno-exceptions # No C++ exceptions
                -fno-rtti # No runtime type info
                -ffunction-sections # Each function in own section
                -fdata-sections # Each data in own section
    )
endif()

# Embedded-specific compile definitions
target_compile_definitions(
    yeet_sfc_embedded PRIVATE # Disable features not suitable for embedded
                              YEET_EMBEDDED_TARGET)

# Create an alias target for consistent naming
add_library(yeet::yeet_sfc_embedded ALIAS yeet_sfc_embedded)
