cmake_minimum_required(VERSION 3.22)

project(yeet_sfc_embedded)

# ==================== Enforce Embedded Build Requirement ====================
if(NOT YEET_BUILD_EMBEDDED)
    message(
        FATAL_ERROR
            "yeet sfc_embedded: This target requires YEET_BUILD_EMBEDDED=ON. "
            "Please configure with: cmake -DYEET_BUILD_EMBEDDED=ON ...")
endif()

# Option to select which side to build
set(YEET_SFC_EMBEDDED_SIDE
    "GRID"
    CACHE STRING "Select which side to build: GRID or MACHINE")
set_property(
    CACHE YEET_SFC_EMBEDDED_SIDE
    PROPERTY STRINGS
             "GRID"
             "MACHINE"
             "BOTH")

# Validate the option
if(NOT
   YEET_SFC_EMBEDDED_SIDE
   MATCHES
   "^(GRID|MACHINE|BOTH)$")
    message(FATAL_ERROR "YEET_SFC_EMBEDDED_SIDE must be GRID, MACHINE, or BOTH")
endif()

message(
    STATUS
        "yeet sfc_embedded: Building targets for ${YEET_SFC_EMBEDDED_SIDE} side"
)

# ==================== Grid-Side Target ====================
if(YEET_SFC_EMBEDDED_SIDE STREQUAL "GRID" OR YEET_SFC_EMBEDDED_SIDE STREQUAL
                                             "BOTH")
    add_library(yeet_sfc_embedded_grid INTERFACE)

    target_sources(
        yeet_sfc_embedded_grid
        INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/grid_side_tasks.cpp>
    )

    target_include_directories(
        yeet_sfc_embedded_grid
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)

    target_link_libraries(
        yeet_sfc_embedded_grid INTERFACE yeet::yeet_control
                                         yeet::yeet_application_sfc_integration)

    # Compile options and definitions are forwarded via INTERFACE They will be
    # applied to any target that links against this library
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
        target_compile_options(
            yeet_sfc_embedded_grid
            INTERFACE -mthumb
                      -mcpu=cortex-m4
                      -mfpu=fpv4-sp-d16
                      -mfloat-abi=hard
                      -fno-exceptions
                      -fno-rtti
                      -ffunction-sections
                      -fdata-sections)
    endif()

    target_compile_definitions(
        yeet_sfc_embedded_grid INTERFACE YEET_EMBEDDED_TARGET
                                         YEET_EMBEDDED_GRID_SIDE)

    add_library(yeet::yeet_sfc_embedded_grid ALIAS yeet_sfc_embedded_grid)

    message(STATUS "  - Grid-side target: yeet_sfc_embedded_grid (INTERFACE) ✅")
endif()

# ==================== Machine-Side Target ====================
if(YEET_SFC_EMBEDDED_SIDE STREQUAL "MACHINE" OR YEET_SFC_EMBEDDED_SIDE STREQUAL
                                                "BOTH")
    add_library(yeet_sfc_embedded_machine INTERFACE)

    target_sources(
        yeet_sfc_embedded_machine
        INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/machine_side_tasks.cpp>
    )

    target_include_directories(
        yeet_sfc_embedded_machine
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)

    target_link_libraries(
        yeet_sfc_embedded_machine
        INTERFACE yeet::yeet_control yeet::yeet_application_sfc_integration)

    # Compile options and definitions are forwarded via INTERFACE
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
        target_compile_options(
            yeet_sfc_embedded_machine
            INTERFACE -mthumb
                      -mcpu=cortex-m4
                      -mfpu=fpv4-sp-d16
                      -mfloat-abi=hard
                      -fno-exceptions
                      -fno-rtti
                      -ffunction-sections
                      -fdata-sections)
    endif()

    target_compile_definitions(
        yeet_sfc_embedded_machine INTERFACE YEET_EMBEDDED_TARGET
                                            YEET_EMBEDDED_MACHINE_SIDE)

    add_library(yeet::yeet_sfc_embedded_machine ALIAS yeet_sfc_embedded_machine)

    message(
        STATUS
            "  - Machine-side target: yeet_sfc_embedded_machine (INTERFACE) ✅")
endif()

# ==================== Legacy Compatibility Alias ====================
if(YEET_SFC_EMBEDDED_SIDE STREQUAL "GRID")
    add_library(yeet::yeet_sfc_embedded ALIAS yeet_sfc_embedded_grid)
elseif(YEET_SFC_EMBEDDED_SIDE STREQUAL "MACHINE")
    add_library(yeet::yeet_sfc_embedded ALIAS yeet_sfc_embedded_machine)
endif()

# ==================== Board Support Configuration ====================
# Include STM32 HAL support for embedded builds
message(
    STATUS "yeet sfc_embedded: Including STM32 HAL support for embedded builds")
include(stm32_hal)

# Add the board directory based on selection The board CMakeLists.txt will
# create the final executables
set(BOARD_DIR "${CMAKE_SOURCE_DIR}/boards/${YEET_SFC_EMBEDDED_BOARD}")
if(NOT EXISTS ${BOARD_DIR})
    message(FATAL_ERROR "Board directory not found: ${BOARD_DIR}")
endif()
add_subdirectory(${BOARD_DIR}
                 ${CMAKE_BINARY_DIR}/boards/${YEET_SFC_EMBEDDED_BOARD})

message(
    STATUS
        "yeet sfc_embedded: Configuration complete. Executables will be created by board target."
)
