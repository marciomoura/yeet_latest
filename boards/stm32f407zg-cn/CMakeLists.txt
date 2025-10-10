cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once, and is not re-generated if converter is
# called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32f407zg-cn)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Setup STM32 drivers symlinks (must happen before stm32cubemx subdirectory)
include(${CMAKE_SOURCE_DIR}/cmake/setup_stm32_drivers.cmake)
setup_stm32_drivers(${CMAKE_CURRENT_SOURCE_DIR})

# ==================== Create Board HAL Library ====================
# This library contains all board-specific hardware drivers and HAL
add_library(${CMAKE_PROJECT_NAME}_hal STATIC)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(
    ${CMAKE_PROJECT_NAME}_hal PRIVATE # Add user defined library search paths
)

# Add board HAL sources to library
target_sources(
    ${CMAKE_PROJECT_NAME}_hal
    PRIVATE # Source files (.c)
            Core/Src/stm32f4xx_it.c
            Core/Src/stm32f4xx_hal_msp.c
            Core/Src/syscalls.c
            Core/Src/sysmem.c
            Core/Src/ad7606_driver.c
            Core/Src/fpga_io_driver.c
            # Header files (.h) - for IDE visibility
            Core/Inc/main.h
            Core/Inc/stm32f4xx_it.h
            Core/Inc/stm32f4xx_hal_conf.h
            Core/Inc/ad7606_driver.h
            Core/Inc/fpga_io_driver.h
            # Note: main.c is NOT included - it's part of the application
            # executable
)

# Add include paths
target_include_directories(
    ${CMAKE_PROJECT_NAME}_hal
    PUBLIC
        Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../../software/yeet_application/sfc_embedded/include
        # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(
    ${CMAKE_PROJECT_NAME}_hal PUBLIC USE_HAL_DRIVER STM32F407xx
                                     # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Note: Libraries are linked by stm32cubemx CMakeLists

# Create alias for easier linking
add_library(${CMAKE_PROJECT_NAME}::hal ALIAS ${CMAKE_PROJECT_NAME}_hal)

# ==================== Legacy Compatibility Alias ====================
# Keep the old target name as an alias for backward compatibility
add_library(${CMAKE_PROJECT_NAME} ALIAS ${CMAKE_PROJECT_NAME}_hal)

# ==================== Create Application Executables ====================
# Create executables for each side of the SFC control (grid and machine) The
# YEET_SFC_EMBEDDED_SIDE variable determines which executables are created

message(
    STATUS
        "${CMAKE_PROJECT_NAME}: Creating application executables based on YEET_SFC_EMBEDDED_SIDE=${YEET_SFC_EMBEDDED_SIDE}"
)

# Grid-Side Executable
if(YEET_SFC_EMBEDDED_SIDE STREQUAL "GRID" OR YEET_SFC_EMBEDDED_SIDE STREQUAL
                                             "BOTH")
    set(GRID_APP_NAME yeet_embedded_app_grid)

    add_executable(${GRID_APP_NAME} Core/Src/main.c)

    target_link_libraries(
        ${GRID_APP_NAME} PRIVATE ${CMAKE_PROJECT_NAME}_hal
                                 yeet::yeet_sfc_embedded_grid m) # Math library

    # Suppress newlib syscall warnings (we provide our own implementations)
    target_link_options(
        ${GRID_APP_NAME}
        PRIVATE
        "LINKER:--no-warn-rwx-segment"
        "LINKER:--no-warn-mismatch")

    # Embedded-specific compile definitions
    target_compile_definitions(
        ${GRID_APP_NAME}
        PRIVATE USE_HAL_DRIVER
                STM32F407xx
                YEET_EMBEDDED_TARGET
                YEET_EMBEDDED_GRID_SIDE)

    message(STATUS "  - Created executable: ${GRID_APP_NAME} 🚀")
endif()

# Machine-Side Executable
if(YEET_SFC_EMBEDDED_SIDE STREQUAL "MACHINE" OR YEET_SFC_EMBEDDED_SIDE STREQUAL
                                                "BOTH")
    set(MACHINE_APP_NAME yeet_embedded_app_machine)

    add_executable(${MACHINE_APP_NAME} Core/Src/main.c)

    target_link_libraries(
        ${MACHINE_APP_NAME} PRIVATE ${CMAKE_PROJECT_NAME}_hal
                                    yeet::yeet_sfc_embedded_machine m) # Math
                                                                       # library

    # Embedded-specific compile definitions
    target_compile_definitions(
        ${MACHINE_APP_NAME}
        PRIVATE USE_HAL_DRIVER
                STM32F407xx
                YEET_EMBEDDED_TARGET
                YEET_EMBEDDED_MACHINE_SIDE)

    message(STATUS "  - Created executable: ${MACHINE_APP_NAME} 🚀")
endif()
