cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once, and is not re-generated if converter is
# called multiple times.
#
# User is free to modify the file as much as necessary
#

# Suppress unused variable warning
set(ignoreMe "${CMAKE_COMMAND}")

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32f407g-disc1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

message("Building embedded board library: ${CMAKE_PROJECT_NAME}")
message("Build type: ${CMAKE_BUILD_TYPE}")

# Create a static library for the board support package
add_library(${CMAKE_PROJECT_NAME} STATIC)

# Add STM32CubeMX generated sources (HAL, CMSIS, etc.) This adds all the
# necessary sources to the stm32f407g-disc1 library
add_subdirectory(cmake/stm32cubemx)

# Add wide character function stubs for newlib-nano compatibility
target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Src/wchar_stubs.c)

# Link the stm32cubemx interface library PUBLIC so consumers get headers
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC stm32cubemx)

# Add include paths - PUBLIC so consumers inherit them
target_include_directories(
    ${CMAKE_PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/USB_HOST/App>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/USB_HOST/Target>
           $<INSTALL_INTERFACE:include>)

# Add project symbols (macros) - PUBLIC so consumers inherit them
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC USE_HAL_DRIVER
                                                        STM32F407xx)

# Linker script is already set in the toolchain file, don't set it again here
# target_link_options(${CMAKE_PROJECT_NAME} INTERFACE
# -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407XX_FLASH.ld)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Optional: Add alias for better namespace management
add_library(board::${CMAKE_PROJECT_NAME} ALIAS ${CMAKE_PROJECT_NAME})
