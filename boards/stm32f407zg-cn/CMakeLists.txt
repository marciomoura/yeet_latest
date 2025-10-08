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

# Create a STATIC LIBRARY instead of executable This allows the application
# layer to link against it
add_library(${CMAKE_PROJECT_NAME} STATIC)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(
    ${CMAKE_PROJECT_NAME} PRIVATE # Add user defined library search paths
)

# Add board sources to library
target_sources(
    ${CMAKE_PROJECT_NAME}
    PRIVATE Core/Src/stm32f4xx_it.c
            Core/Src/stm32f4xx_hal_msp.c
            Core/Src/syscalls.c
            Core/Src/sysmem.c
            # Note: main.c is NOT included here - it will be added by the
            # application executable
)

# Add include paths
target_include_directories(
    ${CMAKE_PROJECT_NAME} PUBLIC Core/Inc
                                 # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(
    ${CMAKE_PROJECT_NAME} PUBLIC USE_HAL_DRIVER STM32F407xx
                                 # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(
    ${CMAKE_PROJECT_NAME} stm32cubemx # Add user defined libraries
)
