cmake_minimum_required(VERSION 3.22)

# ==============================================================================#
# STM32F407ZG-CN Board Support Package This file is NOT auto-generated and
# survives STM32CubeMX regeneration
# ==============================================================================#

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Set the project name
set(CMAKE_PROJECT_NAME stm32f407zg-cn)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Setup C compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

message(STATUS "🎯 Building embedded board library: ${CMAKE_PROJECT_NAME}")
message(STATUS "🔨 Build type: ${CMAKE_BUILD_TYPE}")

# ==============================================================================#
# Check that CMSIS libraries are available
# ==============================================================================#
if(NOT DEFINED CMSIS_DEVICE_DIR)
    message(WARNING "⚠️ CMSIS_DEVICE_DIR not defined, using local CMSIS")
    set(CMSIS_DEVICE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device")
endif()

if(NOT DEFINED CMSIS_CORE_DIR)
    message(WARNING "⚠️ CMSIS_CORE_DIR not defined, using local CMSIS")
    set(CMSIS_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include")
endif()

message(STATUS "✅ Using CMSIS Device from: ${CMSIS_DEVICE_DIR}")
message(STATUS "✅ Using CMSIS Core from: ${CMSIS_CORE_DIR}")

# ==============================================================================#
# Create STM32F4xx Standard Peripheral Driver Library
# ==============================================================================#
add_library(STM32F4xx_StdPeriph_Driver STATIC)

# Collect all Standard Peripheral Driver source files
file(GLOB SPL_SOURCES
     "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_StdPeriph_Driver/src/*.c")

# Add system_stm32f4xx.c from SPL
target_sources(
    STM32F4xx_StdPeriph_Driver
    PRIVATE
        ${SPL_SOURCES}
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_StdPeriph_Driver/src/system_stm32f4xx.c
)

# Set include directories for SPL library
target_include_directories(
    STM32F4xx_StdPeriph_Driver
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_StdPeriph_Driver/inc
           ${CMSIS_DEVICE_DIR}/Include
           ${CMSIS_CORE_DIR}/Include
           ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc)

# Add SPL configuration definitions Note: STM32F407xx must be defined BEFORE
# including CMSIS headers The device header (stm32f407xx.h) defines
# __FPU_PRESENT=1 automatically
target_compile_definitions(
    STM32F4xx_StdPeriph_Driver
    PUBLIC USE_STDPERIPH_DRIVER
           STM32F407xx
           ARM_MATH_CM4 # Enable ARM Cortex-M4 math functions
           __FPU_PRESENT=1 # Explicitly define FPU present (redundant but safe)
           $<$<CONFIG:Debug>:DEBUG>)

message(STATUS "✅ STM32F4xx Standard Peripheral Driver library configured")

# ==============================================================================#
# Create main board support package library
# ==============================================================================#
add_library(${CMAKE_PROJECT_NAME} STATIC)

# Add STM32CubeMX generated application sources (can be regenerated safely)
target_sources(
    ${CMAKE_PROJECT_NAME}
    PRIVATE # Core application (main is excluded, provided by user application)
            # ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c
            # System support
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
            # NSD Driver files - Refactored drivers only (files that actually
            # exist)
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/adc.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/dac.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/digital_ios.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/driver_init.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/fiber_optic.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/fpga.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/fsmc_gpio_driver.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/nand_driver.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/sram_driver.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/tasks.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/thyristor_control.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/w5200_driver.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/network_config_helper.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/watchdog_simple.c
            # Legacy network code (commented out - using new driver now)
            # ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/legacy/w5200.c
            # ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/legacy/net.c
            # ${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/legacy/pub.c Startup
            # file (assembly)
            ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f407xx.s)

# Add include paths - PUBLIC so consumers inherit them
target_include_directories(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/legacy>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_StdPeriph_Driver/inc>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../software/yeet_application/sfc_embedded/include>
        $<INSTALL_INTERFACE:include>)

# Add project symbols (macros) - PUBLIC so consumers inherit them (SPL ONLY)
target_compile_definitions(
    ${CMAKE_PROJECT_NAME}
    PUBLIC USE_STDPERIPH_DRIVER
           STM32F407xx
           ARM_MATH_CM4 # Enable ARM Cortex-M4 math functions
           __FPU_PRESENT=1 # Explicitly define FPU present
           $<$<CONFIG:Debug>:DEBUG>)

# Link the Standard Peripheral Driver library (SPL ONLY)
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC STM32F4xx_StdPeriph_Driver)

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(
    ${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES
                                     ${CMAKE_PROJECT_NAME}.map)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Optional: Add alias for better namespace management
add_library(board::${CMAKE_PROJECT_NAME} ALIAS ${CMAKE_PROJECT_NAME})

message(STATUS "✅ Board library ${CMAKE_PROJECT_NAME} configured successfully")
