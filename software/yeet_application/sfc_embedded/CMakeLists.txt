cmake_minimum_required(VERSION 3.22)

project(yeet_sfc_embedded)

# ==================== Build Mode Selection ====================
option(YEET_SFC_HOST_BUILD
       "Build SFC simulator for host platform (not embedded)" OFF)

# ==================== Enforce Embedded Build Requirement ====================
if(NOT YEET_BUILD_EMBEDDED AND NOT YEET_SFC_HOST_BUILD)
    message(
        FATAL_ERROR
            "yeet sfc_embedded: This target requires either YEET_BUILD_EMBEDDED=ON or YEET_SFC_HOST_BUILD=ON. "
            "Please configure with: cmake -DYEET_BUILD_EMBEDDED=ON ... or cmake -DYEET_SFC_HOST_BUILD=ON ..."
    )
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
if(NOT YEET_SFC_HOST_BUILD)
    if(YEET_SFC_EMBEDDED_SIDE STREQUAL "GRID" OR YEET_SFC_EMBEDDED_SIDE
                                                 STREQUAL "BOTH")
        add_library(yeet_sfc_embedded_grid STATIC)

        target_sources(
            yeet_sfc_embedded_grid
            PRIVATE src/grid_side_tasks.cpp
                    src/fiber_optic_protocol.cpp
                    src/network_telemetry.cpp
                    src/network_telemetry_builder.cpp
                    src/network_command_parser.cpp
                    src/network_telemetry_manager.cpp)

        target_include_directories(
            yeet_sfc_embedded_grid
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)

        target_link_libraries(
            yeet_sfc_embedded_grid
            PUBLIC yeet::yeet_control yeet::yeet_application_sfc_integration)

        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
            target_compile_options(
                yeet_sfc_embedded_grid
                PRIVATE -mthumb
                        -mcpu=cortex-m4
                        -mfpu=fpv4-sp-d16
                        -mfloat-abi=hard
                        -fno-exceptions
                        -fno-rtti
                        -ffunction-sections
                        -fdata-sections)
        endif()

        target_compile_definitions(
            yeet_sfc_embedded_grid PRIVATE YEET_EMBEDDED_TARGET
                                           YEET_EMBEDDED_GRID_SIDE)

        add_library(yeet::yeet_sfc_embedded_grid ALIAS yeet_sfc_embedded_grid)

        message(STATUS "  - Grid-side target: yeet_sfc_embedded_grid ✅")
    endif()
endif()

# ==================== Machine-Side Target ====================
if(NOT YEET_SFC_HOST_BUILD)
    if(YEET_SFC_EMBEDDED_SIDE STREQUAL "MACHINE" OR YEET_SFC_EMBEDDED_SIDE
                                                    STREQUAL "BOTH")
        add_library(yeet_sfc_embedded_machine STATIC)

        target_sources(
            yeet_sfc_embedded_machine PRIVATE src/machine_side_tasks.cpp
                                              src/fiber_optic_protocol.cpp)

        target_include_directories(
            yeet_sfc_embedded_machine
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)

        target_link_libraries(
            yeet_sfc_embedded_machine
            PUBLIC yeet::yeet_control yeet::yeet_application_sfc_integration)

        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
            target_compile_options(
                yeet_sfc_embedded_machine
                PRIVATE -mthumb
                        -mcpu=cortex-m4
                        -mfpu=fpv4-sp-d16
                        -mfloat-abi=hard
                        -fno-exceptions
                        -fno-rtti
                        -ffunction-sections
                        -fdata-sections)
        endif()

        target_compile_definitions(
            yeet_sfc_embedded_machine PRIVATE YEET_EMBEDDED_TARGET
                                              YEET_EMBEDDED_MACHINE_SIDE)

        add_library(yeet::yeet_sfc_embedded_machine ALIAS
                    yeet_sfc_embedded_machine)

        message(STATUS "  - Machine-side target: yeet_sfc_embedded_machine ✅")
    endif()
endif()

# ==================== Legacy Compatibility Alias ====================
if(NOT YEET_SFC_HOST_BUILD)
    if(YEET_SFC_EMBEDDED_SIDE STREQUAL "GRID")
        add_library(yeet::yeet_sfc_embedded ALIAS yeet_sfc_embedded_grid)
    elseif(YEET_SFC_EMBEDDED_SIDE STREQUAL "MACHINE")
        add_library(yeet::yeet_sfc_embedded ALIAS yeet_sfc_embedded_machine)
    endif()
endif()

# ==================== Embedded Board and Executable Configuration
# ====================
if(NOT YEET_SFC_HOST_BUILD)
    message(STATUS "yeet sfc_embedded: Configuring embedded executable 🎯")

    # Include STM32 HAL support
    include(stm32_hal)

    # Add the board directory based on selection - builds the stm32cubemx
    # library
    set(BOARD_DIR "${CMAKE_SOURCE_DIR}/boards/${YEET_SFC_EMBEDDED_BOARD}")
    if(NOT EXISTS ${BOARD_DIR})
        message(FATAL_ERROR "Board directory not found: ${BOARD_DIR}")
    endif()
    add_subdirectory(${BOARD_DIR}
                     ${CMAKE_BINARY_DIR}/boards/${YEET_SFC_EMBEDDED_BOARD})

    # Determine which library to link based on configuration
    if(YEET_SFC_EMBEDDED_SIDE STREQUAL "BOTH")
        message(
            FATAL_ERROR
                "Cannot build single executable with BOTH sides. Build separate executables for GRID and MACHINE."
        )
    elseif(YEET_SFC_EMBEDDED_SIDE STREQUAL "MACHINE")
        set(EMBEDDED_LIBRARY yeet::yeet_sfc_embedded_machine)
        set(EMBEDDED_APP_NAME yeet_embedded_app_machine)
        set(EMBEDDED_SIDE_DEF YEET_EMBEDDED_MACHINE_SIDE)
    else()
        # Default to GRID
        set(EMBEDDED_LIBRARY yeet::yeet_sfc_embedded_grid)
        set(EMBEDDED_APP_NAME yeet_embedded_app_grid)
        set(EMBEDDED_SIDE_DEF YEET_EMBEDDED_GRID_SIDE)
    endif()

    # Create the embedded application executable
    add_executable(${EMBEDDED_APP_NAME} ${BOARD_DIR}/Core/Src/main.c)

    # Link against the board library and selected side
    target_link_libraries(
        ${EMBEDDED_APP_NAME}
        PRIVATE -Wl,--whole-archive
                ${EMBEDDED_LIBRARY}
                ${YEET_SFC_EMBEDDED_BOARD}
                -Wl,--no-whole-archive
                m) # Math library

    # Embedded-specific compile definitions
    target_compile_definitions(
        ${EMBEDDED_APP_NAME}
        PRIVATE USE_HAL_DRIVER
                STM32F407xx
                YEET_EMBEDDED_TARGET
                ${EMBEDDED_SIDE_DEF})

    message(
        STATUS "yeet sfc_embedded: Created executable ${EMBEDDED_APP_NAME} 🚀")
    message(STATUS "yeet sfc_embedded: Linked with ${EMBEDDED_LIBRARY}")
endif() # NOT YEET_SFC_HOST_BUILD

# ==================== Host Simulator Target ====================
if(YEET_SFC_HOST_BUILD)
    message(STATUS "yeet sfc_embedded: Building HOST SIMULATOR 💻")

    # Host simulator executable
    add_executable(sfc_host_simulator)

    target_sources(
        sfc_host_simulator
        PRIVATE # Host platform layer
                src/host_platform/host_main.cpp
                src/host_platform/host_tcp_server.cpp
                src/host_platform/host_timing.cpp
                src/host_platform/host_io_mocks.cpp
                # Actual embedded control code (conditionally compiled)
                src/grid_side_tasks.cpp
                src/fiber_optic_protocol.cpp
                src/network_telemetry.cpp
                src/network_telemetry_builder.cpp
                src/network_command_parser.cpp
                src/network_telemetry_manager.cpp)

    target_include_directories(sfc_host_simulator
                               PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

    target_link_libraries(
        sfc_host_simulator PRIVATE yeet::yeet_control
                                   yeet::yeet_application_sfc_integration)

    # Host-specific compile definitions
    target_compile_definitions(
        sfc_host_simulator
        PRIVATE HOST_BUILD YEET_EMBEDDED_GRID_SIDE # Default to grid side for
                                                   # now
    )

    # Platform-specific linking
    if(WIN32)
        # Windows: Link Winsock2
        target_link_libraries(sfc_host_simulator PRIVATE ws2_32)
    else()
        # Linux: Link pthread for threading
        target_link_libraries(sfc_host_simulator PRIVATE pthread)
    endif()

    message(STATUS "yeet sfc_embedded: Host simulator executable created ✅")
    message(STATUS "  Platform: ${CMAKE_SYSTEM_NAME}")
    message(STATUS "  Usage: ./sfc_host_simulator --port 7000 --frequency 100")
endif()
