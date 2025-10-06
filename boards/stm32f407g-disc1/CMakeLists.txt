cmake_minimum_required(VERSION 3.22)

# ==============================================================================#
# STM32F407G-DISC1 Board Support Package This file is NOT auto-generated and
# survives STM32CubeMX regeneration
# ==============================================================================#

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Set the project name
set(CMAKE_PROJECT_NAME stm32f407g-disc1)

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
# Check that STM32 HAL libraries were fetched by parent CMakeLists
# ==============================================================================#
if(NOT DEFINED STM32_HAL_DIR)
    message(
        FATAL_ERROR
            "❌ STM32_HAL_DIR not defined. Make sure stm32_hal.cmake is included before this file."
    )
endif()

message(STATUS "✅ Using fetched STM32 HAL from: ${STM32_HAL_DIR}")
message(STATUS "✅ Using fetched CMSIS Device from: ${CMSIS_DEVICE_DIR}")
message(STATUS "✅ Using fetched CMSIS Core from: ${CMSIS_CORE_DIR}")
message(STATUS "✅ Using fetched USB Host from: ${STM32_USB_HOST_DIR}")

# ==============================================================================#
# Create STM32 HAL Drivers Library (from fetched sources)
# ==============================================================================#
add_library(STM32_HAL_Drivers STATIC)

# Collect all HAL source files from fetched library
file(GLOB HAL_SOURCES "${STM32_HAL_DIR}/Src/*.c")

# Filter out template files
list(
    FILTER
    HAL_SOURCES
    EXCLUDE
    REGEX
    ".*_template\\.c$")

# Add HAL sources to library
target_sources(STM32_HAL_Drivers PRIVATE ${HAL_SOURCES})

# Add CMSIS system file from local Core (board-specific)
target_sources(STM32_HAL_Drivers
               PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32f4xx.c)

# Set include directories for HAL library
target_include_directories(
    STM32_HAL_Drivers
    PUBLIC ${STM32_HAL_DIR}/Inc
           ${STM32_HAL_DIR}/Inc/Legacy
           ${CMSIS_DEVICE_DIR}/Include
           ${CMSIS_CORE_DIR}/Include
           ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc)

# Add HAL configuration definitions
target_compile_definitions(STM32_HAL_Drivers PUBLIC USE_HAL_DRIVER STM32F407xx
                                                    $<$<CONFIG:Debug>:DEBUG>)

# ==============================================================================#
# Create USB Host Library (from fetched ST Middleware)
# ==============================================================================#
add_library(USB_Host_Library STATIC)

# USB Host Core sources (from fetched library)
target_sources(
    USB_Host_Library
    PRIVATE ${STM32_USB_HOST_DIR}/Core/Src/usbh_core.c
            ${STM32_USB_HOST_DIR}/Core/Src/usbh_ctlreq.c
            ${STM32_USB_HOST_DIR}/Core/Src/usbh_ioreq.c
            ${STM32_USB_HOST_DIR}/Core/Src/usbh_pipes.c
            # USB CDC Class (if needed)
            ${STM32_USB_HOST_DIR}/Class/CDC/Src/usbh_cdc.c)

# USB Host include directories
target_include_directories(
    USB_Host_Library
    PUBLIC ${STM32_USB_HOST_DIR}/Core/Inc
           ${STM32_USB_HOST_DIR}/Class/CDC/Inc
           ${CMAKE_CURRENT_SOURCE_DIR}/USB_HOST/App
           ${CMAKE_CURRENT_SOURCE_DIR}/USB_HOST/Target)

# USB Host depends on HAL
target_link_libraries(USB_Host_Library PUBLIC STM32_HAL_Drivers)

# ==============================================================================#
# Create main board support package library
# ==============================================================================#
add_library(${CMAKE_PROJECT_NAME} STATIC)

# Add STM32CubeMX generated application sources (can be regenerated safely)
target_sources(
    ${CMAKE_PROJECT_NAME}
    PRIVATE # USB Host application
            ${CMAKE_CURRENT_SOURCE_DIR}/USB_HOST/Target/usbh_conf.c
            ${CMAKE_CURRENT_SOURCE_DIR}/USB_HOST/Target/usbh_platform.c
            ${CMAKE_CURRENT_SOURCE_DIR}/USB_HOST/App/usb_host.c
            # Core application (main is excluded, provided by user application)
            # ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c
            # System support
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
            ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/wchar_stubs.c
            # Startup file (assembly)
            ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f407xx.s)

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

# Link the HAL and USB libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC STM32_HAL_Drivers
                                                   USB_Host_Library)

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(
    ${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES
                                     ${CMAKE_PROJECT_NAME}.map)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Optional: Add alias for better namespace management
add_library(board::${CMAKE_PROJECT_NAME} ALIAS ${CMAKE_PROJECT_NAME})

message(STATUS "✅ Board library ${CMAKE_PROJECT_NAME} configured successfully")
