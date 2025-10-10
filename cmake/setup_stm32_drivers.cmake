# ==============================================================================#
# STM32 Drivers Setup Script
#
# This script creates the directory structure expected by STM32CubeMX auto-
# generated CMakeLists files, and populates it with symlinks (or copies on
# Windows) pointing to FetchContent-downloaded libraries.
#
# This approach allows STM32CubeMX files to remain completely untouched and
# auto-regeneratable without any manual intervention.
#
# Usage:
#   include(cmake/setup_stm32_drivers.cmake)
#   setup_stm32_drivers(${CMAKE_CURRENT_SOURCE_DIR})
# ==============================================================================#

function(setup_stm32_drivers BOARD_DIR)
    # Ensure HAL has been fetched
    if(NOT DEFINED STM32_HAL_DIR)
        message(
            FATAL_ERROR
                "STM32_HAL_DIR not defined. Include cmake/dependencies/stm32_hal.cmake first."
        )
    endif()

    set(DRIVERS_DIR "${BOARD_DIR}/Drivers")
    set(MIDDLEWARES_DIR "${BOARD_DIR}/Middlewares")

    message(STATUS "🔧 Setting up STM32 drivers for: ${BOARD_DIR}")
    message(
        STATUS
            "🪟 CMAKE_HOST_WIN32=${CMAKE_HOST_WIN32}, CMAKE_HOST_UNIX=${CMAKE_HOST_UNIX}"
    )

    # ========== Create Drivers directory structure ==========

    # Create Drivers/STM32F4xx_HAL_Driver symlink
    set(HAL_DRIVER_LINK "${DRIVERS_DIR}/STM32F4xx_HAL_Driver")
    if(NOT EXISTS "${HAL_DRIVER_LINK}")
        file(MAKE_DIRECTORY "${DRIVERS_DIR}")

        if(CMAKE_HOST_WIN32)
            # Windows: Directly copy (symlinks require Developer Mode or admin)
            message(
                STATUS "  📋 Copying HAL driver (Windows mode - no symlinks)...")
            file(COPY "${STM32_HAL_DIR}/" DESTINATION "${HAL_DRIVER_LINK}")
            if(EXISTS "${HAL_DRIVER_LINK}/Src/stm32f4xx_hal.c")
                message(STATUS "  ✅ Copied: Drivers/STM32F4xx_HAL_Driver")
            else()
                message(FATAL_ERROR "Failed to copy ${HAL_DRIVER_LINK}")
            endif()
        else()
            # Unix: Create symlink (always works)
            execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
                                    "${STM32_HAL_DIR}" "${HAL_DRIVER_LINK}")
            message(STATUS "  ✅ Symlinked: Drivers/STM32F4xx_HAL_Driver")
        endif()
    else()
        message(STATUS "  ℹ️  Drivers/STM32F4xx_HAL_Driver already exists")
    endif()

    # Create Drivers/CMSIS directory structure
    set(CMSIS_DIR "${DRIVERS_DIR}/CMSIS")
    file(MAKE_DIRECTORY "${CMSIS_DIR}")

    # CMSIS Device symlink
    set(CMSIS_DEVICE_LINK "${CMSIS_DIR}/Device/ST/STM32F4xx")
    if(NOT EXISTS "${CMSIS_DEVICE_LINK}")
        file(MAKE_DIRECTORY "${CMSIS_DIR}/Device/ST")

        if(CMAKE_HOST_WIN32)
            message(STATUS "  📋 Copying: Drivers/CMSIS/Device/ST/STM32F4xx...")
            file(COPY "${CMSIS_DEVICE_DIR}/" DESTINATION "${CMSIS_DEVICE_LINK}")
            if(EXISTS "${CMSIS_DEVICE_LINK}/Include/stm32f4xx.h")
                message(STATUS "  ✅ Copied: Drivers/CMSIS/Device/ST/STM32F4xx")
            else()
                message(FATAL_ERROR "Failed to copy ${CMSIS_DEVICE_LINK}")
            endif()
        else()
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E create_symlink
                        "${CMSIS_DEVICE_DIR}" "${CMSIS_DEVICE_LINK}")
            message(STATUS "  ✅ Symlinked: Drivers/CMSIS/Device/ST/STM32F4xx")
        endif()
    else()
        message(STATUS "  ℹ️  Drivers/CMSIS/Device/ST/STM32F4xx already exists")
    endif()

    # CMSIS Include symlink
    set(CMSIS_INCLUDE_LINK "${CMSIS_DIR}/Include")
    if(NOT EXISTS "${CMSIS_INCLUDE_LINK}")
        if(CMAKE_HOST_WIN32)
            message(STATUS "  📋 Copying: Drivers/CMSIS/Include...")
            file(COPY "${CMSIS_CORE_DIR}/Include/"
                 DESTINATION "${CMSIS_INCLUDE_LINK}")
            if(EXISTS "${CMSIS_INCLUDE_LINK}/core_cm4.h")
                message(STATUS "  ✅ Copied: Drivers/CMSIS/Include")
            else()
                message(FATAL_ERROR "Failed to copy ${CMSIS_INCLUDE_LINK}")
            endif()
        else()
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E create_symlink
                        "${CMSIS_CORE_DIR}/Include" "${CMSIS_INCLUDE_LINK}")
            message(STATUS "  ✅ Symlinked: Drivers/CMSIS/Include")
        endif()
    else()
        message(STATUS "  ℹ️  Drivers/CMSIS/Include already exists")
    endif()

    # ========== Create Middlewares directory (for USB Host) ==========

    if(DEFINED STM32_USB_HOST_DIR)
        set(USB_HOST_LINK "${MIDDLEWARES_DIR}/ST/STM32_USB_Host_Library")
        if(NOT EXISTS "${USB_HOST_LINK}")
            file(MAKE_DIRECTORY "${MIDDLEWARES_DIR}/ST")

            if(CMAKE_HOST_WIN32)
                message(
                    STATUS
                        "  📋 Copying: Middlewares/ST/STM32_USB_Host_Library...")
                file(COPY "${STM32_USB_HOST_DIR}/"
                     DESTINATION "${USB_HOST_LINK}")
                if(EXISTS "${USB_HOST_LINK}/Core/Inc/usbh_core.h")
                    message(
                        STATUS
                            "  ✅ Copied: Middlewares/ST/STM32_USB_Host_Library")
                else()
                    message(FATAL_ERROR "Failed to copy ${USB_HOST_LINK}")
                endif()
            else()
                execute_process(
                    COMMAND ${CMAKE_COMMAND} -E create_symlink
                            "${STM32_USB_HOST_DIR}" "${USB_HOST_LINK}")
                message(
                    STATUS
                        "  ✅ Symlinked: Middlewares/ST/STM32_USB_Host_Library")
            endif()
        else()
            message(
                STATUS
                    "  ℹ️  Middlewares/ST/STM32_USB_Host_Library already exists"
            )
        endif()
    endif()

    message(STATUS "✅ STM32 drivers setup complete for ${BOARD_DIR}")
endfunction()


